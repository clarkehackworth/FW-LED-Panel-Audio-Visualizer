#!/usr/bin/env python3
"""
Framework LED Matrix Audio Visualizer
======================================
Captures system audio playback and streams real-time FFT bar-graph data to
one or two Framework LED matrix panels.

Requires custom firmware built with firmware/setup.sh.

Platform support:
  Linux  — auto-detects PulseAudio/PipeWire monitor source via pactl
  Windows — auto-detects default output device via WASAPI loopback
              (set audio.wasapi_loopback: false in config to use mic instead)

Usage:
    python3 audio_viz.py [--config config.yaml] [options]
    python3 audio_viz.py --list-devices   # show audio devices and exit
    python3 audio_viz.py --left /dev/ttyACM0 --right /dev/ttyACM1
    python3 audio_viz.py --list-devices   # Windows: look for OUTPUT device names

Protocol (custom firmware commands):
    SetEQConfig [0x32 0xAC 0x1D <flags> <fade_min>]
        flags bit 0: 0=bars extend left-to-right, 1=right-to-left
        flags bit 1: 0=low-freq at top row, 1=low-freq at bottom row
        fade_min: brightness at bar tip (0-255, default 60)
    DisplayEQ   [0x32 0xAC 0x21 <h0..h33> <p0..p33> <b0..b33>]
        h_i: bar length     0-9 cols  (one bar per row, 34 bars total)
        p_i: peak dot       0=none, 1-9=col position from bar base
        b_i: peak brightness 0-255, nibble-packed (optional, default 255)
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import serial
import sounddevice as sd
import yaml

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

MAGIC = bytes([0x32, 0xAC])
CMD_SET_EQ_CONFIG = 0x1D
CMD_DISPLAY_EQ = 0x21

PANEL_WIDTH = 9    # firmware cols — bar height axis, 0-9 (physical top-bottom)
PANEL_HEIGHT = 34  # firmware rows — frequency band axis, 0-33 (physical right-left)
NUM_BANDS = PANEL_HEIGHT   # one frequency band per firmware row
BAR_MAX   = PANEL_WIDTH    # max bar height value sent to firmware


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------

class Panel:
    """Manages the serial connection and protocol for a single LED panel."""

    def __init__(self, port: str, flags: int, fade_min: int) -> None:
        self.port = port
        # flags bit 0: 0=left-to-right, 1=right-to-left
        # flags bit 1: 0=low-freq at top, 1=low-freq at bottom
        self._flags = flags & 0x03
        self._fade_min = max(0, min(255, fade_min))
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        self._ser = serial.Serial(self.port, 115200, timeout=0.1, write_timeout=0.05)
        cfg_cmd = MAGIC + bytes([CMD_SET_EQ_CONFIG, self._flags, self._fade_min])
        self._ser.write(cfg_cmd)
        ext = "right-to-left" if self._flags & 0x01 else "left-to-right"
        freq = "bottom" if self._flags & 0x02 else "top"
        print(f"  Panel {self.port}: connected (bars {ext}, low-freq at {freq})")

    @staticmethod
    def _pack_nibbles(values: list[int]) -> bytes:
        """Pack NUM_BANDS values (0-15 each) into 17 bytes, low nibble first."""
        out = bytearray(17)
        for i, v in enumerate(values[:NUM_BANDS]):
            if i % 2 == 0:
                out[i // 2] = v & 0x0F
            else:
                out[i // 2] |= (v & 0x0F) << 4
        return bytes(out)

    def send_eq(self, heights: list[int], peaks: list[int], peak_brightness: list[int]) -> None:
        h = [max(0, min(BAR_MAX, v)) for v in heights[:NUM_BANDS]]
        p = [max(0, min(BAR_MAX, v)) for v in peaks[:NUM_BANDS]]
        # Convert brightness 0-255 → nibble 0-15; firmware decodes as nibble * 17
        b = [max(0, min(15, v // 17)) for v in peak_brightness[:NUM_BANDS]]
        # Pad to exactly 34 values then nibble-pack → 17 bytes each (54 bytes total)
        h += [0] * (NUM_BANDS - len(h))
        p += [0] * (NUM_BANDS - len(p))
        b += [0] * (NUM_BANDS - len(b))
        cmd = MAGIC + bytes([CMD_DISPLAY_EQ]) + self._pack_nibbles(h) + self._pack_nibbles(p) + self._pack_nibbles(b)
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.write(cmd)
            except serial.SerialException as exc:
                print(f"  Warning: serial write to {self.port} failed: {exc}")

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()


# ---------------------------------------------------------------------------
# Peak tracker (host-side)
# ---------------------------------------------------------------------------

class PeakTracker:
    """Tracks per-band peak indicators with hold, fall-off, and brightness fade."""

    def __init__(self, n_bars: int, hold_frames: int, fall_speed: float, fade_speed: float) -> None:
        self._peaks = np.zeros(n_bars, dtype=float)
        self._hold = np.zeros(n_bars, dtype=int)
        self._brightness = np.zeros(n_bars, dtype=float)
        self._hold_frames = hold_frames
        self._fall_speed = fall_speed    # pixels per frame
        self._fade_speed = fade_speed    # brightness units per frame (0-255 scale)

    def update(self, bar_heights: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (peak_positions, peak_brightnesses), each clipped to valid range."""
        risen = bar_heights > self._peaks
        self._peaks[risen] = bar_heights[risen]
        self._hold[risen] = self._hold_frames
        self._brightness[risen] = 255.0  # new peak: snap to full brightness

        falling = ~risen
        self._hold[falling] = np.maximum(0, self._hold[falling] - 1)
        should_fall = falling & (self._hold == 0)
        self._peaks[should_fall] = np.maximum(0.0, self._peaks[should_fall] - self._fall_speed)

        # Fade brightness for all peaks that didn't just rise this frame
        self._brightness[~risen] = np.maximum(0.0, self._brightness[~risen] - self._fade_speed)

        return self._peaks.astype(int), np.clip(self._brightness, 0, 255).astype(int)


# ---------------------------------------------------------------------------
# Frequency bin builder
# ---------------------------------------------------------------------------

def make_log_bins(n_bars: int, freq_min: float, freq_max: float,
                  fft_size: int, sample_rate: int) -> list[np.ndarray]:
    """
    Return a list of boolean masks (one per bar) mapping FFT bins to frequency bands.
    Bands are spaced logarithmically between freq_min and freq_max.
    """
    edges = np.logspace(np.log10(freq_min), np.log10(freq_max), n_bars + 1)
    freqs = np.fft.rfftfreq(fft_size, d=1.0 / sample_rate)
    masks = []
    for i in range(n_bars):
        mask = (freqs >= edges[i]) & (freqs < edges[i + 1])
        if not mask.any():
            # Band is narrower than one FFT bin — snap to nearest bin
            center = (edges[i] + edges[i + 1]) / 2.0
            nearest = int(np.argmin(np.abs(freqs - center)))
            mask = np.zeros(len(freqs), dtype=bool)
            mask[nearest] = True
        masks.append(mask)
    return masks


# ---------------------------------------------------------------------------
# Main visualizer
# ---------------------------------------------------------------------------

class AudioVisualizer:
    def __init__(self, cfg: dict) -> None:
        viz = cfg.get("visualization", {})
        audio = cfg.get("audio", {})

        self._num_bars: int = min(NUM_BANDS, int(viz.get("num_bars", NUM_BANDS)))
        self._freq_min: float = float(viz.get("freq_min", 60))
        self._freq_max: float = float(viz.get("freq_max", 16000))
        self._db_floor: float = float(viz.get("db_floor", -60))
        self._db_ceil: float = float(viz.get("db_ceiling", -10))
        self._scale: float = float(viz.get("scale", 1.0))
        self._decay: float = float(viz.get("decay", 0.75))
        self._attack: float = float(viz.get("attack", 0.9))
        self._target_fps: int = int(viz.get("target_fps", 30))
        self._peaks_enabled: bool = bool(viz.get("peaks", True))
        peak_hold: int = int(viz.get("peak_hold_frames", 30))
        peak_fall: float = float(viz.get("peak_fall_speed", 1.0))
        peak_fade: float = float(viz.get("peak_fade_speed", 5.0))
        self._bar_fade_min: int = int(viz.get("bar_fade_min", 60))

        self._sample_rate: int = int(audio.get("sample_rate", 44100))
        self._fft_size: int = int(audio.get("fft_size", 1024))
        self._source = audio.get("source", "auto")
        # WASAPI loopback: capture system output on Windows. Defaults to True on
        # Windows, False elsewhere. Override with audio.wasapi_loopback in config.
        default_loopback = sys.platform == "win32"
        self._wasapi_loopback: bool = bool(audio.get("wasapi_loopback", default_loopback))

        # Pre-computed state
        self._bass_skip: int = int(viz.get("bass_skip", 6))

        self._hann = np.hanning(self._fft_size).astype(np.float32)
        all_masks = make_log_bins(
            self._num_bars + self._bass_skip, self._freq_min, self._freq_max,
            self._fft_size, self._sample_rate,
        )
        self._bin_masks = all_masks[self._bass_skip:]
        self._smooth = np.zeros(self._num_bars, dtype=float)
        self._peak_tracker = PeakTracker(self._num_bars, peak_hold, peak_fall, peak_fade)

        # Audio ring buffer (filled by callback, drained by main thread)
        self._ring = np.zeros(self._fft_size, dtype=np.float32)
        self._audio_q: queue.Queue[np.ndarray] = queue.Queue(maxsize=8)

        # Panels
        ext_map  = {"left": 0, "right": 1}   # bar extension direction
        freq_map = {"top": 0, "bottom": 2}    # low-freq position (bit 1)
        self._panels: list[Panel] = []
        for side in ("left_panel", "right_panel"):
            pcfg = cfg.get(side, {})
            port = pcfg.get("port")
            if not port:
                continue
            flags  = ext_map.get(pcfg.get("direction", "left"), 0)
            flags |= freq_map.get(pcfg.get("freq_start", "top"), 0)
            if pcfg.get("mirror", False):
                flags ^= 0x01  # flip bar direction
            self._panels.append(Panel(port, flags, self._bar_fade_min))

    # ------------------------------------------------------------------
    # Audio callback — runs in a separate C thread; keep it minimal
    # ------------------------------------------------------------------

    def _audio_callback(self, indata: np.ndarray, frames: int, time_info, status) -> None:
        mono = indata[:, 0] if indata.ndim > 1 else indata.ravel()
        try:
            self._audio_q.put_nowait(mono.copy())
        except queue.Full:
            pass  # drop chunk — main loop is behind, visual glitch is acceptable

    # ------------------------------------------------------------------
    # FFT + bar computation
    # ------------------------------------------------------------------

    def _process(self, samples: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (bar_lengths, peak_lengths), each an int array clipped to [0, BAR_MAX]."""
        windowed = samples * self._hann
        spectrum = np.abs(np.fft.rfft(windowed, n=self._fft_size))
        db = 20.0 * np.log10(spectrum + 1e-10)

        raw = np.full(self._num_bars, self._db_floor, dtype=float)
        for i, mask in enumerate(self._bin_masks):
            if mask.any():
                raw[i] = db[mask].mean()

        # Map dB range to [0, 1]
        normalized = np.clip((raw - self._db_floor) / (self._db_ceil - self._db_floor), 0.0, 1.0)

        # Attack-fast / decay-slow smoothing
        rising = normalized > self._smooth
        self._smooth[rising] = (self._attack * normalized[rising]
                                + (1.0 - self._attack) * self._smooth[rising])
        self._smooth[~rising] *= self._decay

        # Bars use gamma curve; peaks stay linear so they always reach full brightness
        bars  = np.clip((self._smooth ** self._scale * BAR_MAX).astype(int), 0, BAR_MAX)
        peaks = np.clip((self._smooth * BAR_MAX).astype(int), 0, BAR_MAX)
        return bars, peaks

    # ------------------------------------------------------------------
    # Device discovery
    # ------------------------------------------------------------------

    def _find_device(self):
        if self._source == "default":
            return None
        if self._source != "auto":
            try:
                return int(self._source)
            except ValueError:
                return self._source  # sounddevice accepts name substrings

        if self._wasapi_loopback:
            return self._find_device_wasapi_loopback()

        return self._find_device_monitor()

    def _find_device_wasapi_loopback(self) -> Optional[int]:
        """Return the default WASAPI output device index for loopback capture."""
        try:
            default_out = sd.default.device[1]
            if default_out is not None and default_out >= 0:
                name = sd.query_devices(default_out)["name"]
                print(f"  Auto-selected WASAPI loopback: {name}")
                return int(default_out)
        except Exception:
            pass

        # Fallback: first device with output channels
        for dev in sd.query_devices():
            if dev["max_output_channels"] > 0:
                print(f"  Auto-selected WASAPI loopback: {dev['name']}")
                return int(dev["index"])

        print("  Warning: no WASAPI output device found — falling back to default input")
        print("  Tip: run --list-devices to see device names, then set audio.source in config.yaml")
        self._wasapi_loopback = False
        return None

    def _find_device_monitor(self) -> Optional[object]:
        """Find a PulseAudio/PipeWire monitor source (Linux/macOS)."""
        devices = sd.query_devices()

        # First: look for a monitor source in sounddevice's device list
        for dev in devices:
            name: str = dev["name"].lower()
            if dev["max_input_channels"] > 0 and "monitor" in name:
                print(f"  Auto-selected audio device: {dev['name']}")
                return dev["index"]

        # Second: enumerate all PulseAudio/PipeWire sources via pactl and probe each
        try:
            import subprocess
            result = subprocess.run(
                ["pactl", "list", "short", "sources"],
                capture_output=True, text=True, timeout=2,
            )
            monitor_sources = [
                line.split()[1]
                for line in result.stdout.splitlines()
                if len(line.split()) >= 2 and ".monitor" in line.split()[1]
            ]
            for source in monitor_sources:
                for rate in (self._sample_rate, 48000, 44100):
                    try:
                        sd.check_input_settings(device=source, channels=1, samplerate=rate)
                        print(f"  Auto-selected monitor source: {source}")
                        return source
                    except Exception:
                        continue
        except Exception:
            pass

        print("  Warning: no usable monitor source found — falling back to default input (microphone)")
        print("  Tip: run --list-devices to see sounddevice inputs, or set audio.source in config.yaml")
        return None

    # ------------------------------------------------------------------
    # Sample-rate negotiation
    # ------------------------------------------------------------------

    def _resolve_sample_rate(self, device) -> int:
        """Return the sample rate to use, falling back to the device's native rate."""
        wanted = self._sample_rate
        try:
            # WASAPI loopback uses the output device's native rate; can't negotiate.
            kind = "output" if self._wasapi_loopback else "input"
            info = sd.query_devices(device, kind=kind)
            native = int(info["default_samplerate"])
        except Exception:
            return wanted

        if native == wanted:
            return wanted

        # For regular input, verify the wanted rate is actually supported first.
        if not self._wasapi_loopback:
            try:
                sd.check_input_settings(device=device, channels=1, samplerate=wanted)
                return wanted
            except Exception:
                pass

        # Rebuild FFT state for the native rate
        print(f"  Sample rate {wanted} Hz not supported — using device native {native} Hz")
        self._sample_rate = native
        self._ring = np.zeros(self._fft_size, dtype=np.float32)
        all_masks = make_log_bins(
            self._num_bars + self._bass_skip, self._freq_min, self._freq_max,
            self._fft_size, native,
        )
        self._bin_masks = all_masks[self._bass_skip:]
        self._hann = np.hanning(self._fft_size).astype(np.float32)
        return native

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        if not self._panels:
            print("ERROR: no panels configured. Set left_panel.port / right_panel.port in config.yaml")
            sys.exit(1)

        print("Connecting to panels...")
        for panel in self._panels:
            panel.connect()

        device = self._find_device()
        sample_rate = self._resolve_sample_rate(device)
        frame_interval = 1.0 / self._target_fps

        extra = sd.WasapiSettings(loopback=True) if self._wasapi_loopback else None
        try:
            with sd.InputStream(
                device=device,  # int index, name string, or None for default
                channels=1,
                samplerate=sample_rate,
                blocksize=256,          # small blocks = low latency
                dtype=np.float32,
                callback=self._audio_callback,
                extra_settings=extra,
            ):
                print(f"Streaming at {self._target_fps} fps. Ctrl-C to stop.")
                last_frame = time.monotonic()

                while True:
                    # Accumulate audio into ring buffer
                    try:
                        chunk = self._audio_q.get(timeout=0.2)
                    except queue.Empty:
                        continue

                    n = len(chunk)
                    self._ring = np.roll(self._ring, -n)
                    self._ring[-n:] = chunk

                    now = time.monotonic()
                    if now - last_frame < frame_interval:
                        continue
                    last_frame = now

                    bars, peaks = self._process(self._ring)

                    # Pad to full panel height (34 frequency bands)
                    h_full = [0] * NUM_BANDS
                    p_full = [0] * NUM_BANDS
                    b_full = [0] * NUM_BANDS
                    for i in range(self._num_bars):
                        h_full[i] = int(bars[i])

                    if self._peaks_enabled:
                        peaks_arr, peak_bright = self._peak_tracker.update(peaks)
                        for i in range(self._num_bars):
                            p_full[i] = int(peaks_arr[i])
                            b_full[i] = int(peak_bright[i])

                    for panel in self._panels:
                        panel.send_eq(h_full, p_full, b_full)

        except KeyboardInterrupt:
            print("\nStopping...")
            zeros = [0] * NUM_BANDS
            for panel in self._panels:
                panel.send_eq(zeros, zeros, zeros)
        finally:
            for panel in self._panels:
                panel.close()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Framework LED Matrix Audio Visualizer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("-c", "--config", default="config.yaml",
                   help="Path to YAML config file (default: config.yaml)")
    p.add_argument("--left", metavar="PORT",
                   help="Left panel serial port (e.g. /dev/ttyACM0)")
    p.add_argument("--right", metavar="PORT",
                   help="Right panel serial port (e.g. /dev/ttyACM1)")
    p.add_argument("--left-dir", choices=["left", "right"], default=None,
                   help="Left panel bar extension direction (default from config)")
    p.add_argument("--right-dir", choices=["left", "right"], default=None,
                   help="Right panel bar extension direction (default from config)")
    p.add_argument("--left-freq", choices=["top", "bottom"], default=None,
                   help="Left panel low-frequency position (default from config)")
    p.add_argument("--right-freq", choices=["top", "bottom"], default=None,
                   help="Right panel low-frequency position (default from config)")
    p.add_argument("--bars", type=int, metavar="N",
                   help="Number of frequency bars 1-34 (default from config)")
    p.add_argument("--fps", type=int, metavar="N",
                   help="Target frame rate (default from config)")
    p.add_argument("--no-peaks", action="store_true",
                   help="Disable peak indicators")
    p.add_argument("--list-devices", action="store_true",
                   help="List audio input devices and exit")
    p.add_argument("--device", metavar="INDEX_OR_NAME",
                   help="Audio input device index or substring of name")
    p.add_argument("--clear", action="store_true",
                   help="Send an all-zero frame to each panel (blanks the display) and exit")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    # Load config file (optional)
    cfg_path = Path(args.config)
    if cfg_path.exists():
        with open(cfg_path) as f:
            cfg: dict = yaml.safe_load(f) or {}
    else:
        if args.config != "config.yaml":
            print(f"WARNING: config file not found: {cfg_path}")
        cfg = {}

    # CLI overrides
    if args.left:
        cfg.setdefault("left_panel", {})["port"] = args.left
    if args.right:
        cfg.setdefault("right_panel", {})["port"] = args.right
    if args.left_dir:
        cfg.setdefault("left_panel", {})["direction"] = args.left_dir
    if args.right_dir:
        cfg.setdefault("right_panel", {})["direction"] = args.right_dir
    if args.left_freq:
        cfg.setdefault("left_panel", {})["freq_start"] = args.left_freq
    if args.right_freq:
        cfg.setdefault("right_panel", {})["freq_start"] = args.right_freq
    if args.bars:
        cfg.setdefault("visualization", {})["num_bars"] = args.bars
    if args.fps:
        cfg.setdefault("visualization", {})["target_fps"] = args.fps
    if args.no_peaks:
        cfg.setdefault("visualization", {})["peaks"] = False
    if args.device:
        cfg.setdefault("audio", {})["source"] = args.device

    if args.clear:
        zeros = [0] * NUM_BANDS
        for side in ("left_panel", "right_panel"):
            port = cfg.get(side, {}).get("port")
            if not port:
                continue
            panel = Panel(port, flags=0, fade_min=0)
            try:
                panel.connect()
                panel.send_eq(zeros, zeros, zeros)
            finally:
                panel.close()
        return

    AudioVisualizer(cfg).run()


if __name__ == "__main__":
    main()
