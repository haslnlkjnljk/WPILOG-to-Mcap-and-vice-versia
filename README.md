# WPILOG to MCAP

Gets FRC robot data into [Foxglove](https://foxglove.dev) / MCAP, two ways:

- **`serve`**: subscribe to a robot's or simulator's NetworkTables and livestream everything on
  it over a Foxglove WebSocket server, optionally recording it to `.mcap` at the same time.
- **`convert`**: turn a finished `.wpilog` file (WPILib DataLog / AdvantageKit) into an `.mcap`
  file.

```
 robot / simulator                 this tool                      you
┌──────────────────┐  NT4    ┌───────────────────┐  WebSocket  ┌──────────┐
│ NetworkTables    │ ──────▶ │ wpi2mcap serve    │ ──────────▶ │ Foxglove │
└──────────────────┘         │                   │             └──────────┘
┌──────────────────┐         │                   │  --record   ┌──────────┐
│ log.wpilog       │ ──────▶ │ wpi2mcap convert  │ ──────────▶ │ .mcap    │
└──────────────────┘         └───────────────────┘             └──────────┘
```

## Contents

- [Setup](#setup)
- [Livestream from a robot or simulator](#livestream-from-a-robot-or-simulator)
- [Convert a log file](#convert-a-log-file)
- [Output format](#output-format)
- [Troubleshooting](#troubleshooting)
- [Limitations](#limitations)
- [How it works](#how-it-works)
- [What has been tested](#what-has-been-tested)

## Setup

Requires Python 3.10 or newer (developed on 3.14, macOS).

```sh
./setup.sh
```

This creates a virtual environment in `.venv` and installs the dependencies into it:

| Package | Used for |
|---|---|
| [`foxglove-sdk`](https://pypi.org/project/foxglove-sdk/) | the WebSocket server and writing MCAP files |
| [`pyntcore`](https://pypi.org/project/pyntcore/) | the NetworkTables (NT4) client |

The `./wpi2mcap` script runs the tool inside that environment, so there is nothing to activate.
If you prefer, `.venv/bin/python wpiToMCAP.py ...` does the same thing.

## Livestream from a robot or simulator

Pick where the data comes from:

```sh
./wpi2mcap serve --team 766          # the robot; your computer must be on its network
./wpi2mcap serve --sim               # a simulator running on this computer
./wpi2mcap serve --host 10.7.66.2    # any NetworkTables server, as ADDRESS[:PORT]
```

`--team` tries the standard robot addresses for that team number (`10.TE.AM.2`,
`roboRIO-TEAM-FRC.local`, and the USB address `172.22.11.2`). When the connection is made it
prints `NetworkTables connected`.

Then view the data:

1. Open Foxglove.
2. Choose **Open connection > Foxglove WebSocket**.
3. Connect to `ws://localhost:8765`.

Stop with Ctrl-C.

### Options

| Option | Default | Meaning |
|---|---|---|
| `--team N` / `--sim` / `--host ADDRESS[:PORT]` | (one is required) | where to find NetworkTables |
| `--port PORT` | `8765` | port the Foxglove server listens on |
| `--bind ADDRESS` | `127.0.0.1` | address the Foxglove server listens on; use `0.0.0.0` to let other computers connect |
| `--record OUT.mcap` | off | also save everything that is served to an MCAP file |

### Recording while streaming

```sh
./wpi2mcap serve --team 766 --record match1.mcap
```

The file is finished when you stop the tool with Ctrl-C (or `kill`). An existing file with the
same name is overwritten.

### What the robot has to do

`serve` can only show what the robot publishes to NetworkTables. With AdvantageKit that means the
robot code adds an `NT4Publisher` data receiver:

```java
Logger.addDataReceiver(new NT4Publisher());
```

Logged values then appear under `/AdvantageKit/...`, for example
`/AdvantageKit/RealOutputs/<name>`. Anything else on NetworkTables (SmartDashboard, Preferences,
Shuffleboard, ...) is served too.

## Convert a log file

```sh
./wpi2mcap convert path/to/log.wpilog            # writes path/to/log.mcap
./wpi2mcap convert log.wpilog -o out.mcap        # choose the output name
./wpi2mcap convert logs/*.wpilog -o converted/   # writes converted/<name>.mcap for each
```

Open the resulting `.mcap` in Foxglove with **Open local file**. A log that was cut off
mid-write (the robot lost power) is converted up to its last complete record.

## Output format

Each NetworkTables topic / wpilog entry becomes one channel with the same name. Messages are
JSON with a JSON schema, so Foxglove can plot them without any extra setup.

| Source type | Message |
|---|---|
| `boolean`, `int`/`int64`, `float`, `double`, `string`, `json` | `{"value": 1.5}` |
| arrays of those | `{"value": [1.0, 2.0, 3.0]}` |
| `struct:Name` | one field per struct member, e.g. `{"translation": {"x": 1.5, "y": 2.5}, "rotation": {"value": 0.7}}` |
| `struct:Name[]` | `{"value": [{...}, {...}]}` |
| `raw`, `msgpack`, `protobuf`, undecodable structs | `{"value": "<base64>"}` |

Structs are decoded using the schemas the robot publishes alongside its data
(`/.schema/struct:Name`); nested structs and fixed-size array members are supported. In
Foxglove, plot a struct member with a message path such as
`/AdvantageKit/RealOutputs/Shooter/Motor.statorCurrentAmps`.

Each channel also carries the original type string in its metadata (`wpilog_type`).

**Timestamps** are the robot's own clock (microseconds since the robot program started),
converted to nanoseconds. In `serve` mode this is the NetworkTables server time of each value,
so a live recording lines up with the `.wpilog` the robot writes for the same run.

## Troubleshooting

**It never prints `NetworkTables connected`.**
Check that you can reach the robot: `ping 10.7.66.2` (for team 766) should reply, and if the
Driver Station on the same computer shows communications you are on the right network. Try the
address directly with `--host 10.7.66.2`, or `--host 172.22.11.2` over USB. For `--sim`, the
simulator has to be running first.

**It connects, but there are no `/AdvantageKit/...` topics.**
The robot code isn't publishing them; see [What the robot has to do](#what-the-robot-has-to-do).
Some robot code only adds the `NT4Publisher` when not connected to the field.

**`Failed to bind port: Address already in use`.**
Another copy of the tool (or another Foxglove server) is using the port. Stop it, or pick a
different one with `--port`.

**Foxglove connects but a topic shows no data.**
Values are only sent when they change. The latest value is re-sent when you subscribe, so a
blank topic usually means it has never been published.

**`No virtual environment yet`.**
Run `./setup.sh`.

## Limitations

- `NaN` and infinite values become `null`, because JSON can't represent them.
- Structs that use bitfields or enums are passed through as base64 rather than decoded.
- `serve` only sees values published after it connects, plus the current value of each topic.
  It can't go back in time; use `convert` on the robot's `.wpilog` for the full run.
- When a Foxglove client subscribes to a topic, `serve` re-sends that topic's latest value. With
  `--record`, those re-sent values appear in the file as duplicate messages.
- `convert` reads `.wpilog` only. Going the other way (MCAP to wpilog) is not implemented.

## How it works

Everything is in [`wpiToMCAP.py`](wpiToMCAP.py):

- `WpilogParser` reads the wpilog binary format incrementally into `(entry, timestamp, payload)`
  records.
- `StructDecoder` parses WPILib struct schemas (`double x;double y`) and unpacks struct bytes
  into dictionaries.
- `Bridge` takes values from either source, creates a Foxglove channel per topic the first time
  it sees one, and logs each value to it as JSON. Values that arrive before their struct schema
  are held until the schema shows up.
- `convert` feeds a wpilog file through the bridge into an MCAP writer.
- `serve` subscribes to every NetworkTables topic (asking for every value, not just the latest)
  and feeds them through the same bridge into a WebSocket server, and into an MCAP writer if
  `--record` is given.

## What has been tested

- `serve --sim` against real robot code running in the WPILib simulator: 167 topics streamed at
  up to 200 Hz, with message times matching the robot's `/AdvantageKit/Timestamp` exactly.
- `serve` struct decoding and late-joining clients, against a stand-in NetworkTables server
  publishing the way AdvantageKit does.
- `convert` on a hand-built `.wpilog` covering primitives, arrays, strings, nested structs and
  struct arrays.

Not yet tested: a physical robot, struct topics from real robot code, and `convert` on a
`.wpilog` written by a real robot.
