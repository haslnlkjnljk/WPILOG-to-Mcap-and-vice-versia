#!/usr/bin/env python3
"""Turn robot data (WPILib .wpilog files and live NetworkTables) into MCAP / Foxglove data.

Two modes:

    wpiToMCAP.py convert LOG.wpilog [-o OUT.mcap]
        Convert finished log file(s) to MCAP file(s).

    wpiToMCAP.py serve (--team 766 | --sim | --host ADDRESS) [--port 8765] [--record OUT.mcap]
        Subscribe to a robot's (or the simulator's) NetworkTables and serve everything on it
        live over a Foxglove WebSocket server. Open Foxglove and connect to ws://localhost:8765.

Each wpilog entry / NetworkTables topic becomes one channel with the same name. Messages are
JSON: primitives become {"value": ...}, and struct values are decoded into an object with one
field per struct member.
"""

import argparse
import base64
import json
import math
import re
import signal
import struct
import sys
import threading
import time
from pathlib import Path

import foxglove
import ntcore

CONTROL_START = 0
CONTROL_FINISH = 1

STRUCT_PREFIX = "struct:"
SCHEMA_ENTRY_PREFIX = "/.schema/struct:"

# How many messages to hold for an entry whose struct schema hasn't been logged yet.
MAX_PENDING_PER_ENTRY = 10000

# WPILib struct primitive types -> (struct module format char, JSON schema type).
STRUCT_PRIMITIVES = {
    "bool": ("?", "boolean"),
    "char": ("c", "string"),
    "int8": ("b", "integer"),
    "int16": ("h", "integer"),
    "int32": ("i", "integer"),
    "int64": ("q", "integer"),
    "uint8": ("B", "integer"),
    "uint16": ("H", "integer"),
    "uint32": ("I", "integer"),
    "uint64": ("Q", "integer"),
    "float": ("f", "number"),
    "float32": ("f", "number"),
    "double": ("d", "number"),
    "float64": ("d", "number"),
}

# wpilog scalar types -> (struct module format, JSON schema type).
SCALAR_TYPES = {
    "boolean": ("<?", "boolean"),
    "int64": ("<q", "integer"),
    "float": ("<f", "number"),
    "double": ("<d", "number"),
}

# NetworkTables types whose values ntcore hands us already decoded -> JSON schema type.
# (Everything else -- structs, raw, ... -- arrives as bytes, same as in a wpilog.)
NT_DECODED_TYPES = {
    "boolean": "boolean",
    "int": "integer",
    "float": "number",
    "double": "number",
    "string": "string",
    "json": "string",
}

RAW_SCHEMA = {"type": "string", "contentEncoding": "base64"}


# ---------------------------------------------------------------------------------------------
# wpilog parsing
# ---------------------------------------------------------------------------------------------


class WpilogParser:
    """Incremental wpilog parser: feed() it bytes as they arrive, get complete records back."""

    def __init__(self):
        self._buffer = bytearray()
        self._header_done = False

    def feed(self, data):
        """Returns a list of (entry_id, timestamp_us, payload) for the records completed."""
        self._buffer += data
        buf = self._buffer
        pos = 0
        if not self._header_done:
            if len(buf) < 12:
                return []
            if buf[:6] != b"WPILOG":
                raise ValueError("not a wpilog file (bad magic)")
            (extra_header_len,) = struct.unpack_from("<I", buf, 8)
            if len(buf) < 12 + extra_header_len:
                return []
            pos = 12 + extra_header_len
            self._header_done = True

        records = []
        while pos < len(buf):
            bitfield = buf[pos]
            id_len = (bitfield & 0x3) + 1
            size_len = ((bitfield >> 2) & 0x3) + 1
            ts_len = ((bitfield >> 4) & 0x7) + 1
            header_len = 1 + id_len + size_len + ts_len
            if pos + header_len > len(buf):
                break
            p = pos + 1
            entry_id = int.from_bytes(buf[p : p + id_len], "little")
            p += id_len
            size = int.from_bytes(buf[p : p + size_len], "little")
            p += size_len
            timestamp = int.from_bytes(buf[p : p + ts_len], "little")
            p += ts_len
            if p + size > len(buf):
                break
            records.append((entry_id, timestamp, bytes(buf[p : p + size])))
            pos = p + size
        del buf[:pos]
        return records


def parse_start_record(payload):
    """Returns (entry_id, name, type, metadata) from a Start control record."""
    (entry_id,) = struct.unpack_from("<I", payload, 1)
    pos = 5
    strings = []
    for _ in range(3):
        (length,) = struct.unpack_from("<I", payload, pos)
        pos += 4
        strings.append(payload[pos : pos + length].decode("utf-8", "replace"))
        pos += length
    return entry_id, *strings


# ---------------------------------------------------------------------------------------------
# Decoding payloads into JSON-able objects
# ---------------------------------------------------------------------------------------------


class MissingSchema(Exception):
    """The schema for a struct hasn't been logged (yet)."""


class UnsupportedSchema(Exception):
    pass


class StructDecoder:
    """Decodes one WPILib struct type, given the schemas of all known struct types."""

    def __init__(self, name, schemas):
        self.name = name
        # List of (field name, format char or nested StructDecoder, array length or None).
        self.fields = []
        if name not in schemas:
            raise MissingSchema(name)
        for declaration in schemas[name].split(";"):
            declaration = declaration.strip()
            if not declaration:
                continue
            if ":" in declaration or declaration.startswith("enum"):
                raise UnsupportedSchema(f"bitfields/enums not supported (struct {name})")
            match = re.fullmatch(r"(\S+)\s+(\w+)(?:\s*\[(\d+)\])?", declaration)
            if not match:
                raise UnsupportedSchema(f"can't parse '{declaration}' in struct {name}")
            type_name, field_name, array_len = match.groups()
            if type_name in STRUCT_PRIMITIVES:
                field_type = STRUCT_PRIMITIVES[type_name][0]
            else:
                field_type = StructDecoder(type_name, schemas)
            self.fields.append((field_name, field_type, int(array_len) if array_len else None))
        self.size = sum(
            self._field_size(field_type) * (array_len or 1)
            for _, field_type, array_len in self.fields
        )

    @staticmethod
    def _field_size(field_type):
        if isinstance(field_type, StructDecoder):
            return field_type.size
        return struct.calcsize("<" + field_type)

    def decode(self, payload, pos=0):
        result = {}
        for field_name, field_type, array_len in self.fields:
            values = []
            for _ in range(array_len or 1):
                if isinstance(field_type, StructDecoder):
                    values.append(field_type.decode(payload, pos))
                else:
                    (value,) = struct.unpack_from("<" + field_type, payload, pos)
                    values.append(value)
                pos += self._field_size(field_type)
            if array_len is None:
                result[field_name] = values[0]
            elif field_type == "c":
                result[field_name] = b"".join(values).rstrip(b"\0").decode("utf-8", "replace")
            else:
                result[field_name] = values
        return result

    def json_schema(self):
        properties = {}
        for field_name, field_type, array_len in self.fields:
            if isinstance(field_type, StructDecoder):
                schema = field_type.json_schema()
            else:
                json_type = next(j for f, j in STRUCT_PRIMITIVES.values() if f == field_type)
                schema = {"type": json_type}
            if array_len is not None and field_type != "c":
                schema = {"type": "array", "items": schema}
            properties[field_name] = schema
        return {"type": "object", "properties": properties}


def value_schema(schema):
    return {"type": "object", "properties": {"value": schema}}


def raw_value(payload):
    return {"value": base64.b64encode(payload).decode("ascii")}


def decode_string_array(payload):
    (count,) = struct.unpack_from("<I", payload)
    pos = 4
    strings = []
    for _ in range(count):
        (length,) = struct.unpack_from("<I", payload, pos)
        pos += 4
        strings.append(payload[pos : pos + length].decode("utf-8", "replace"))
        pos += length
    return {"value": strings}


def make_decoder(type_name, struct_schemas, from_bytes=True):
    """Returns (decode function: value -> JSON-able object, JSON schema) for a wpilog/NT type.

    Values are the raw payload bytes, except that with from_bytes=False (NetworkTables) the
    types in NT_DECODED_TYPES, and arrays of them, are already Python values.

    Raises MissingSchema if the type is a struct whose schema isn't in struct_schemas yet.
    """
    if not from_bytes:
        is_array = type_name.endswith("[]")
        element_type = type_name[:-2] if is_array else type_name
        if element_type in NT_DECODED_TYPES:
            schema = {"type": NT_DECODED_TYPES[element_type]}
            if is_array:
                schema = {"type": "array", "items": schema}
            return (lambda value: {"value": value}), value_schema(schema)

    if type_name in SCALAR_TYPES:
        fmt, json_type = SCALAR_TYPES[type_name]
        return (
            lambda payload: {"value": struct.unpack_from(fmt, payload)[0]},
            value_schema({"type": json_type}),
        )

    if type_name.endswith("[]") and type_name[:-2] in SCALAR_TYPES:
        fmt, json_type = SCALAR_TYPES[type_name[:-2]]
        size = struct.calcsize(fmt)

        def decode_array(payload):
            if len(payload) % size != 0:
                return raw_value(payload)
            return {"value": [v for (v,) in struct.iter_unpack(fmt, payload)]}

        return decode_array, value_schema({"type": "array", "items": {"type": json_type}})

    if type_name == "string[]":
        return decode_string_array, value_schema({"type": "array", "items": {"type": "string"}})

    if type_name.startswith(STRUCT_PREFIX):
        struct_name = type_name[len(STRUCT_PREFIX) :]
        is_array = struct_name.endswith("[]")
        if is_array:
            struct_name = struct_name[:-2]
        try:
            decoder = StructDecoder(struct_name, struct_schemas)
        except UnsupportedSchema as ex:
            print(f"warning: {ex}; writing raw bytes instead", file=sys.stderr)
            return raw_value, value_schema(RAW_SCHEMA)
        if not is_array:
            return decoder.decode, decoder.json_schema()

        def decode_struct_array(payload):
            return {
                "value": [
                    decoder.decode(payload, pos)
                    for pos in range(0, len(payload) - decoder.size + 1, decoder.size)
                ]
            }

        return decode_struct_array, value_schema({"type": "array", "items": decoder.json_schema()})

    if type_name in ("string", "json", "structschema"):
        return (
            lambda payload: {"value": payload.decode("utf-8", "replace")},
            value_schema({"type": "string"}),
        )

    # raw, msgpack, protobuf, or anything else we don't understand.
    return raw_value, value_schema(RAW_SCHEMA)


def sanitize(value):
    """JSON has no NaN/Infinity (e.g. the current draw of a motor that can't report it)."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: sanitize(v) for k, v in value.items()}
    if isinstance(value, list):
        return [sanitize(v) for v in value]
    return value


# ---------------------------------------------------------------------------------------------
# wpilog records / NT values -> Foxglove channels (which feed an MCAP file and/or the server)
# ---------------------------------------------------------------------------------------------


class Entry:
    def __init__(self, name, type_name, metadata="", from_bytes=True):
        self.name = name
        self.type_name = type_name
        self.metadata = metadata
        self.from_bytes = from_bytes
        self.channel = None
        self.decode = None
        # (timestamp_us, payload) received before this entry's struct schema was logged.
        self.pending = []


class Bridge:
    """Feeds wpilog records or NetworkTables values into Foxglove channels on a context."""

    def __init__(self, context):
        self.context = context
        self.entries = {}  # wpilog entry id, or NT topic name -> Entry
        self.struct_schemas = {}  # struct name -> schema string
        self.schema_entry_ids = {}  # key in self.entries -> struct name
        self.channels = {}  # (topic, type) -> foxglove.Channel
        self.last_message = {}  # channel id -> (data, log_time_ns)
        self.message_count = 0
        # Guards last_message/channel logging; the server's subscribe callback runs on
        # another thread.
        self.lock = threading.Lock()

    def handle_record(self, entry_id, timestamp_us, payload):
        """Handles one record from a wpilog file."""
        if entry_id == 0:
            self._handle_control(payload)
            return
        if entry_id in self.entries:
            self._handle_value(entry_id, timestamp_us, payload)

    def handle_nt_value(self, name, type_name, timestamp_us, value):
        """Handles one value update from NetworkTables."""
        entry = self.entries.get(name)
        if entry is None or entry.type_name != type_name:
            self.entries[name] = Entry(name, type_name, from_bytes=False)
            self.schema_entry_ids.pop(name, None)
            if type_name == "structschema" and name.startswith(SCHEMA_ENTRY_PREFIX):
                self.schema_entry_ids[name] = name[len(SCHEMA_ENTRY_PREFIX) :]
        self._handle_value(name, timestamp_us, value)

    def _handle_value(self, entry_id, timestamp_us, payload):
        entry = self.entries[entry_id]

        if entry_id in self.schema_entry_ids:
            self.struct_schemas[self.schema_entry_ids[entry_id]] = payload.decode(
                "utf-8", "replace"
            )
            self._retry_pending()

        if entry.channel is None and not self._open(entry):
            if len(entry.pending) < MAX_PENDING_PER_ENTRY:
                entry.pending.append((timestamp_us, payload))
            return
        self._log(entry, timestamp_us, payload)

    def finish(self):
        """Call at the end of the log: writes out anything still waiting for a schema."""
        for entry in self.entries.values():
            if entry.channel is None and entry.pending:
                print(
                    f"warning: no schema was logged for {entry.type_name} ({entry.name}); "
                    "writing raw bytes instead",
                    file=sys.stderr,
                )
                self._open(entry, force_raw=True)
                self._flush_pending(entry)

    def resend_latest(self, channel_id):
        """Re-logs a channel's latest message, so a client that just subscribed sees a value.

        Needed because AdvantageKit only logs a value when it changes: without this, anything
        that isn't changing right now would stay blank for a newly connected client.
        """
        with self.lock:
            latest = self.last_message.get(channel_id)
            if latest is None:
                return
            for channel in self.channels.values():
                if channel.id() == channel_id:
                    channel.log(latest[0], log_time=latest[1])

    def _handle_control(self, payload):
        if not payload:
            return
        if payload[0] == CONTROL_START:
            entry_id, name, type_name, metadata = parse_start_record(payload)
            self.entries[entry_id] = Entry(name, type_name, metadata)
            if type_name == "structschema" and name.startswith(SCHEMA_ENTRY_PREFIX):
                self.schema_entry_ids[entry_id] = name[len(SCHEMA_ENTRY_PREFIX) :]
        elif payload[0] == CONTROL_FINISH:
            (entry_id,) = struct.unpack_from("<I", payload, 1)
            self.entries.pop(entry_id, None)
            self.schema_entry_ids.pop(entry_id, None)

    def _open(self, entry, force_raw=False):
        """Creates the entry's decoder and channel. Returns False if its schema is missing."""
        if force_raw:
            entry.decode, json_schema = raw_value, value_schema(RAW_SCHEMA)
            entry.from_bytes = True
        else:
            try:
                entry.decode, json_schema = make_decoder(
                    entry.type_name, self.struct_schemas, entry.from_bytes
                )
            except MissingSchema:
                return False
        key = (entry.name, entry.type_name)
        if key not in self.channels:
            self.channels[key] = foxglove.Channel(
                entry.name,
                message_encoding="json",
                schema=foxglove.Schema(
                    name=entry.type_name,
                    encoding="jsonschema",
                    data=json.dumps(json_schema).encode(),
                ),
                metadata={"wpilog_type": entry.type_name, "wpilog_metadata": entry.metadata},
                context=self.context,
            )
        entry.channel = self.channels[key]
        return True

    def _retry_pending(self):
        for entry in self.entries.values():
            if entry.channel is None and entry.pending and self._open(entry):
                self._flush_pending(entry)

    def _flush_pending(self, entry):
        pending, entry.pending = entry.pending, []
        for timestamp_us, payload in pending:
            self._log(entry, timestamp_us, payload)

    def _log(self, entry, timestamp_us, payload):
        try:
            message = entry.decode(payload)
        except (struct.error, IndexError):
            message = raw_value(payload)
        data = json.dumps(sanitize(message)).encode()
        # wpilog timestamps are microseconds; Foxglove/MCAP want nanoseconds.
        log_time = timestamp_us * 1000
        with self.lock:
            entry.channel.log(data, log_time=log_time)
            self.last_message[entry.channel.id()] = (data, log_time)
        self.message_count += 1


# ---------------------------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------------------------


def convert(wpilog_path, mcap_path):
    """Converts one wpilog file to one MCAP file. Returns the number of messages written."""
    context = foxglove.Context()
    bridge = Bridge(context)
    parser = WpilogParser()
    with foxglove.open_mcap(mcap_path, allow_overwrite=True, context=context):
        with open(wpilog_path, "rb") as log:
            while chunk := log.read(1 << 20):
                for record in parser.feed(chunk):
                    bridge.handle_record(*record)
        bridge.finish()
    return bridge.message_count


class SubscribeListener(foxglove.websocket.ServerListener):
    def __init__(self, bridge):
        self.bridge = bridge

    def on_subscribe(self, client, channel):
        self.bridge.resend_latest(channel.id)


def serve(args):
    nt = ntcore.NetworkTableInstance.getDefault()
    if args.team is not None:
        nt.setServerTeam(args.team)
        source = f"team {args.team}"
    else:
        source = "127.0.0.1" if args.sim else args.host
        host, _, port = source.partition(":")
        nt.setServer(host, int(port or 0))  # Port 0 means the default NT4 port.
    nt.startClient4("wpiToMCAP")

    # sendAll + keepDuplicates: we want every value the robot publishes, not just the latest
    # one each time the network flushes.
    subscriber = ntcore.MultiSubscriber(
        nt, [""], ntcore.PubSubOptions(sendAll=True, keepDuplicates=True, periodic=0.01)
    )
    poller = ntcore.NetworkTableListenerPoller(nt)
    poller.addListener(subscriber, ntcore.EventFlags.kValueAll | ntcore.EventFlags.kImmediate)

    context = foxglove.Context()
    bridge = Bridge(context)
    server = foxglove.start_server(
        name=f"NetworkTables: {source}",
        host=args.bind,
        port=args.port,
        context=context,
        server_listener=SubscribeListener(bridge),
    )
    recorder = None
    if args.record is not None:
        recorder = foxglove.open_mcap(args.record, allow_overwrite=True, context=context)

    print(f"connecting to NetworkTables ({source})...")
    print(f"serving on ws://{args.bind}:{server.port} (Foxglove: Open connection > WebSocket)")

    # Shut down cleanly (finishing the --record file) on `kill` as well as Ctrl-C.
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    connected = False
    try:
        while True:
            if nt.isConnected() != connected:
                connected = not connected
                print("NetworkTables connected" if connected else "NetworkTables disconnected")
            events = poller.readQueue()
            if not events:
                time.sleep(0.005)
                continue
            for event in events:
                if not event.is_(ntcore.EventFlags.kValueAll):
                    continue
                value = event.data.value
                # Server time is the robot's clock (same timebase as its wpilog); it's 0 for
                # values that didn't come from the server.
                timestamp_us = value.server_time() or value.time()
                bridge.handle_nt_value(
                    event.data.topic.getName(),
                    event.data.topic.getTypeString(),
                    timestamp_us,
                    value.value(),
                )
    except KeyboardInterrupt:
        pass
    finally:
        bridge.finish()
        if recorder is not None:
            recorder.close()
        server.stop()
        nt.stopClient()
    print(f"\nserved {bridge.message_count} messages")


def run_convert(args):
    output_is_dir = args.output is not None and (len(args.wpilogs) > 1 or args.output.is_dir())
    if output_is_dir:
        args.output.mkdir(parents=True, exist_ok=True)
    for wpilog in args.wpilogs:
        if args.output is None:
            mcap_path = wpilog.with_suffix(".mcap")
        elif output_is_dir:
            mcap_path = args.output / (wpilog.stem + ".mcap")
        else:
            mcap_path = args.output
        count = convert(wpilog, mcap_path)
        print(f"{wpilog} -> {mcap_path} ({count} messages)")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    commands = parser.add_subparsers(dest="command", required=True)

    convert_parser = commands.add_parser("convert", help="convert .wpilog file(s) to .mcap")
    convert_parser.add_argument("wpilogs", nargs="+", type=Path, help=".wpilog file(s)")
    convert_parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="output .mcap file (single input only) or directory; "
        "defaults to next to each input",
    )
    convert_parser.set_defaults(run=run_convert)

    serve_parser = commands.add_parser(
        "serve", help="subscribe to NetworkTables and serve it live to Foxglove over WebSocket"
    )
    source = serve_parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--team", type=int, help="connect to this team's robot, e.g. 766")
    source.add_argument("--sim", action="store_true", help="connect to a simulator on this PC")
    source.add_argument(
        "--host", metavar="ADDRESS[:PORT]", help="connect to this NetworkTables server"
    )
    serve_parser.add_argument(
        "--bind", default="127.0.0.1", help="address the Foxglove server listens on"
    )
    serve_parser.add_argument("--port", type=int, default=8765, help="Foxglove server port")
    serve_parser.add_argument(
        "--record", type=Path, metavar="OUT.mcap", help="also record everything served to MCAP"
    )
    serve_parser.set_defaults(run=serve)

    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
