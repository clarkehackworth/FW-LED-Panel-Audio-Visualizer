# Framework LED Matrix Audio Visualizer

Real-time FFT frequency-bar visualizer for the Framework 16 LED input-module
panels.  Both left and right panels display the same audio spectrum; the bar
direction on each side is mirrored and configurable at runtime.

```
Left panel (← bars)        Right panel (bars →)
┌─────────┐                ┌─────────┐
│░░░░░████│ low            │████░░░░░│ low
│░░░░░░░██│                │██░░░░░░░│
│░████████│                │████████░│
│░░░░█████│                │█████░░░░│
│░░░░░░███│                │███░░░░░░│
│░░░░░░░░░│ high           │░░░░░░░░░│ high
└─────────┘                └─────────┘
```

## How it works

- **Host** (`host/audio_viz.py`) captures system audio playback (PulseAudio/
  PipeWire on Linux; WASAPI loopback on Windows), computes an FFT, maps it to
  up to 34 log-spaced frequency bands, and sends bar heights + peaks + brightness
  to each panel over USB serial (~54 bytes per panel per frame).
- **Firmware** (patched `inputmodule-rs`) receives two new commands
  (`SetEQConfig` / `DisplayEQ`) and renders the bar graph directly on the panel,
  handling the direction logic in hardware.  The host sends only heights; the
  panel draws them.  This is ~26× less data than sending full column pixel data.

## Quick start

### 1 — Flash custom firmware

```bash
cd firmware
bash setup.sh          # installs toolchain, clones repo, patches, builds
```

Then follow the flash instructions printed at the end.  Repeat for both panels.

### 2 — Install host dependencies

```bash
cd host
pip install -r requirements.txt
```

`sounddevice` requires PortAudio.  Install the system library if `pip install` fails:

```bash
sudo dnf install portaudio-devel   # Fedora
sudo apt install libportaudio2     # Debian/Ubuntu
# Windows: no extra step — PortAudio is bundled with the sounddevice wheel
```

### 3 — Configure

Edit `host/config.yaml`:

- Set `left_panel.port` and `right_panel.port` to your panel devices.
  - **Linux**: find them with `ls /dev/ttyACM*` (plug/unplug to identify which is which).
  - **Windows**: find them in Device Manager → Ports (COM & LPT) — look for `COMx`.
- Set `left_panel.direction` and `right_panel.direction` (`left` or `right`).
- Adjust `num_bars`, `freq_min/max`, `decay`, `attack`, `target_fps` to taste.

#### Audio source

**Linux** — `audio.source: auto` finds the first PipeWire/PulseAudio `.monitor`
source automatically.  Run `--list-devices` to see inputs; set `audio.source` to
an index or name substring if auto-detection picks the wrong one.

**Windows** — `audio.source: auto` uses WASAPI loopback on the default output
device (speakers/headphones), capturing whatever is playing back.  Run
`--list-devices` and look at the **output** device list if you need to target a
specific device.  To capture a microphone instead, set `audio.wasapi_loopback: false`.

```bash
python3 host/audio_viz.py --list-devices
```

### 4 — Run

```bash
# Linux
python3 host/audio_viz.py
python3 host/audio_viz.py --left /dev/ttyACM0 --right /dev/ttyACM1

# Windows
python host/audio_viz.py
python host/audio_viz.py --left COM3 --right COM4
```

### CLI flags

| Flag | Description |
|---|---|
| `-c FILE` | Config file path (default: `config.yaml`) |
| `--left PORT` | Left panel serial port |
| `--right PORT` | Right panel serial port |
| `--left-dir left\|right` | Override left bar extension direction |
| `--right-dir left\|right` | Override right bar extension direction |
| `--left-freq top\|bottom` | Override left panel low-frequency position |
| `--right-freq top\|bottom` | Override right panel low-frequency position |
| `--bars N` | Number of frequency bars (1-34) |
| `--fps N` | Target frame rate |
| `--no-peaks` | Disable peak-hold indicators |
| `--device IDX` | Audio device index or name substring from `--list-devices` |
| `--list-devices` | Print audio devices and exit |
| `--clear` | Blank both panels and exit |

## Firmware protocol

Two new commands are added to the existing USB ACM serial protocol
(`[0x32, 0xAC, cmd, ...]`):

| Command | ID | Payload | Description |
|---|---|---|---|
| `SetEQConfig` | `0x1D` | `[flags, fade_min]` | Configure display (flags bit 0: bar direction; bit 1: freq orientation; fade_min: tip brightness 0-255) |
| `DisplayEQ` | `0x21` | `[h0..h33, p0..p33, b0..b33]` | Bar heights + peak positions + peak brightness, each nibble-packed to 17 bytes (51 bytes total) |

`SetEQConfig` is sent once on connect.
`DisplayEQ` is sent every frame (54 bytes total: 2 magic + 1 cmd + 51 data).

## Project layout

```
ledpanels/
├── firmware/
│   ├── setup.sh          # clone, patch, build, flash instructions
│   ├── patch.py          # applies our command additions to inputmodule-rs
│   └── inputmodule-rs/   # created by setup.sh
└── host/
    ├── audio_viz.py      # main script
    ├── config.yaml       # runtime configuration
    └── requirements.txt
```

## Tuning tips

- **Bass-heavy response**: lower `freq_min` to 40 Hz, raise `db_ceiling` to -5.
- **Snappy attack**: set `attack: 0.95`, `decay: 0.6`.
- **Smooth, glowing decay**: `attack: 0.85`, `decay: 0.88`.
- **No peaks, cleaner look**: `peaks: false` or `--no-peaks`.
- **7 bars with gaps**: `num_bars: 7` — the two rightmost columns will stay dark.
