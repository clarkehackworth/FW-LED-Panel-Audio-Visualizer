#!/usr/bin/env python3
"""
Framework LED Matrix Audio Visualizer
======================================
Captures system audio playback and streams real-time FFT bar-graph data to
one or two Framework LED matrix panels.

Requires custom firmware built with firmware/setup.sh.

Platform support:
  Linux  — auto-detects PipeWire tap sources (e.g. "Easy Effects Sink",
               "Equalizer (Speakers)") for system audio capture
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

import glob

import os

import queue

import sys

import threading

import time

from contextlib import contextmanager

from pathlib import Path

from typing import Optional

import numpy as np

import serial

import sounddevice as sd

import yaml


# ---------------------------------------------------------------------------
# PortAudio/ALSA error suppression
# ---------------------------------------------------------------------------
# PortAudio writes error messages directly to C-level stderr (fd 2) via
# vfprintf(stderr, ...), which bypasses Python's sys.stderr entirely.
# We must redirect at the raw file descriptor level using os.dup2.

@contextmanager
def _suppress_stderr():
    """Silence PortAudio/ALSA errors at the C-level stderr (fd 2)."""
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    old_fd2 = os.dup(2)
    try:
        os.dup2(devnull_fd, 2)
        yield
    finally:
        os.dup2(old_fd2, 2)
        os.close(devnull_fd)
        os.close(old_fd2)


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
        self._ser = serial.Serial(self.port, 115200, timeout=0.1, write_timeout=2.0)
        # Flush any stale data from the device
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        cfg_cmd = MAGIC + bytes([CMD_SET_EQ_CONFIG, self._flags, self._fade_min])
        # Retry config handshake up to 3 times
        for attempt in range(3):
            try:
                self._ser.write(cfg_cmd)
                # Small delay to let device process the config
                self._ser.in_waiting  # force read-check
                break
            except serial.SerialException as exc:
                if attempt < 2:
                    print(f"  Warning: config write to {self.port} attempt {attempt+1} failed: {exc} — retrying…")
                    self._ser.reset_input_buffer()
                    self._ser.reset_output_buffer()
                else:
                    print(f"  Warning: config write to {self.port} failed after 3 attempts: {exc}")
        ext = "right-to-left" if self._flags & 0x01 else "left-to-right"
        freq = "bottom" if self._flags & 0x02 else "top"
        print(f"  Panel {self.port}: connected (bars {ext}, low-freq at {freq})")

    @staticmethod
    def _pack_nibbles(values: list[int]) -> bytes:
        """Pack NUM_BANDS values (0-15 each) into 17 bytes, low nibble first."""
        arr = np.array(values[:NUM_BANDS], dtype=np.uint8) & 0x0F
        return (arr[0::2] | (arr[1::2] << 4)).tobytes()

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
            if self._ser and self._ser.is_open:
                # Retry up to 3 times on timeout; skip warnings after first failure to avoid spam
                for attempt in range(3):
                    try:
                        self._ser.write(cmd)
                        break
                    except serial.SerialException as exc:
                        if attempt < 2:
                            print(f"  Warning: serial write to {self.port} attempt {attempt+1} failed: {exc} — retrying…")
                            time.sleep(0.05)  # brief pause before retry
                        else:
                            print(f"  Warning: serial write to {self.port} failed after 3 attempts: {exc}")

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
# Microphone detection helpers
# ---------------------------------------------------------------------------

_MICROPHONE_PATTERNS = [
    "mic",
    "microphone",
    "alsa_input",
    "pulse_input",
]

# Internal processing nodes and other visualizers that should never be used as
# an audio source.  "easy effects filter" is an intermediate EQ chain node
# (not the final output), and "spectrum" is another audio analyzer that would
# create a feedback loop if used as our source.
_EXCLUDED_DEVICE_PATTERNS = [
    "easy effects filter",
    "easyeffects filter",
    "spectrum",
]

def _is_microphone(name: str) -> bool:
    """Return True if *name* looks like a microphone / capture device."""
    lower = name.lower()
    for pat in _MICROPHONE_PATTERNS:
        if pat in lower:
            return True
    # Also flag devices that are purely "Default" input without "loopback" or "tap"
    if lower == "default" or lower.startswith("default input"):
        return True
    return False

def _is_excluded_device(name: str) -> bool:
    """Return True if *name* is a known bad candidate (internal node or other visualizer)."""
    lower = name.lower()
    return any(pat in lower for pat in _EXCLUDED_DEVICE_PATTERNS)


# ---------------------------------------------------------------------------
# PortAudio re-init helper (forces device list refresh)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Audio device monitor (background auto-switching)
# ---------------------------------------------------------------------------

class _ProbeDevice:
    """Monitors a single audio input device for signal energy."""

    def __init__(self, device, sample_rate: int, fft_size: int) -> None:
        self.device = device
        self._sample_rate = sample_rate
        self._fft_size = fft_size
        self._device_name = ""
        self._lock = threading.Lock()
        self._energy = 0.0
        self._updated = 0.0
        self._last_active = 0.0  # monotonic time of last energy > threshold

    # -- probing via short-lived stream --------------------------------

    def probe(self) -> float:
        """Open a short-lived stream, read energy, return RMS level.

        Runs in a background thread with a 5-second join timeout so we
        don't hang on devices that can't be opened.  Blocks until the
        probe finishes or times out.
        """
        buf = []
        done = threading.Event()
        result = {"energy": 0.0}

        def _cb(indata, frames, time_info, status):
            if status and "input overflow" not in status and "output underflow" not in status:
                print(f"  [probe] status for {self.device}: {status}")
            if indata.ndim < 2 or indata.shape[1] < 1:
                return
            buf.append(indata[:, 0].copy())

        def _resolve_device():
            """Resolve a device identifier (index or name) to what sounddevice accepts."""
            dev = self.device
            if isinstance(dev, int):
                return dev
            # String name — verify it matches a real PortAudio input device.
            for d in sd.query_devices():
                if d["max_input_channels"] == 0:
                    continue
                dname = d["name"]
                if dname is None:
                    continue
                if isinstance(dev, str) and dev in dname.lower():
                    return d["index"]
            print(f"  [probe] device name '{dev}' not found in PortAudio devices")
            return None

        def _do_probe():
            stream = None
            opened = False
            try:
                dev = _resolve_device()
                if dev is None:
                    return

                with _suppress_stderr():
                    # Try stereo first (for PipeWire monitor sources),
                    # then mono (for regular hardware inputs).
                    for channels in (2, 1):
                        for rate in (self._sample_rate, 48000, 44100, 48001, 44101):
                            if stream is not None:
                                try:
                                    stream.stop()
                                    stream.close()
                                except Exception:
                                    pass
                            try:
                                stream = sd.InputStream(
                                    device=dev,
                                    channels=channels,
                                    samplerate=rate,
                                    blocksize=256,
                                    dtype=np.float32,
                                    callback=_cb,
                                )
                                stream.start()
                                opened = True
                                break
                            except Exception:
                                stream = None  # type: ignore[assignment]
                        if opened:
                            break
                    else:
                        print(f"  [probe] failed to open device {self.name} (stereo+mono)")
                        return

                    time.sleep(0.5)

                    stream.stop()
                    stream.close()
                    stream = None

                    if buf:
                        samples = np.concatenate(buf)
                        energy = float(np.sqrt(np.mean(samples ** 2)))
                        now = time.monotonic()
                        with self._lock:
                            self._energy = energy
                            self._updated = now
                            if energy > 1e-6:
                                self._last_active = now
                        result["energy"] = energy
                        db = 20.0 * np.log10(energy) if energy > 1e-10 else -100.0
                        print(f"  [probe] {self.name}: {energy:.6f} RMS ({db:.1f} dB)")
            except Exception as e:
                print(f"  [probe] error on {self.name}: {e}")
            finally:
                done.set()

        t = threading.Thread(target=_do_probe, daemon=True)
        t.start()
        t.join(timeout=5.0)  # wait up to ~5.5 s total
        return result["energy"]

    @property
    def energy(self) -> float:
        with self._lock:
            return self._energy

    @property
    def updated(self) -> float:
        with self._lock:
            return self._updated

    @property
    def name(self) -> str:
        return self._device_name

    @name.setter
    def name(self, value: str) -> None:
        self._device_name = value


class AudioMonitor:
    """
    Background monitor that listens on multiple audio devices and detects
    which one has the strongest audio signal.  Used to auto-switch the
    main visualizer stream when a louder source becomes available.

    Periodically refreshes the device list to pick up new devices or
    remove devices that disappeared.
    """

    def __init__(
        self,
        probe_devices: list[_ProbeDevice],
        current_device,
        sample_rate: int,
        threshold: float = 1e-3,       # ~ -60 dBFS
        hysteresis_db: float = 6.0,    # dB margin before switching
        persistence_timeout: float = 60.0,  # seconds of silence before considering switch
        refresh_interval: float = 30.0,     # seconds between device list refreshes
        allow_mics: bool = False,           # include microphone candidates
        probe_map: Optional[dict] = None,   # shared map updated by refresh
        fft_size: int = 1024,              # used to create new ProbeDevice instances
    ) -> None:
        self._probes = probe_devices
        self._current = current_device
        self._sample_rate = sample_rate
        self._fft_size = fft_size
        self._threshold = threshold
        self._hysteresis_db = hysteresis_db
        self._persistence_timeout = persistence_timeout
        self._refresh_interval = refresh_interval
        self._allow_mics = allow_mics
        self._probe_map = probe_map
        self._switch_event: Optional[threading.Event] = None

    # -- helpers ----------------------------------------------------------

    @staticmethod
    def _rms_to_db(rms: float) -> float:
        """Convert RMS (0-1) to dBFS."""
        if rms < 1e-10:
            return -100.0
        return 20.0 * np.log10(rms)

    @staticmethod
    def _db_to_linear(db: float) -> float:
        """Convert dBFS back to RMS."""
        return 10.0 ** (db / 20.0)

    # -- probing ----------------------------------------------------------

    def _probe_all(self) -> None:
        """Probe every non-current device via short-lived streams."""
        current_idx = self._current_index
        for i, probe in enumerate(self._probes):
            if i == current_idx:
                continue  # skip the device we're already listening on
            probe.probe()

  # -- device list refresh ----------------------------------------------

    def _refresh_devices(self, probe_map: dict) -> tuple[bool, set]:
        """Re-query the system for audio devices and rebuild the probe list.

        Returns (True if the probe list changed, set of new device indices).
        """
        # 1. Query PortAudio devices
        devices = sd.query_devices()

        # 2. Build candidates from PortAudio
        pa_new_candidates: list[tuple[object, str]] = []
        for dev in devices:
            if dev["max_input_channels"] == 0:
                continue
            name_lower = dev["name"].lower()
            if name_lower in ("pipewire", "default"):
                continue
            if not self._allow_mics and _is_microphone(dev["name"]):
                continue
            if _is_excluded_device(dev["name"]):
                continue
            pa_new_candidates.append((dev["index"], dev["name"]))

        # Build a set of candidate device indices (excluding the current one).
        current_idx = self._current_index
        current_device = self._probes[current_idx].device if self._probes else None

        # Deduplicate candidates
        seen_indices: set = set()
        new_candidates = []
        for idx, name in pa_new_candidates:
            if idx not in seen_indices:
                seen_indices.add(idx)
                new_candidates.append((idx, name))
        

        # Separate current device from candidates.
        new_probes: list[_ProbeDevice] = []
        candidates_added = False
        new_device_indices: set = set()

        # Check if current device still exists in the candidate list.
        # None means "sounddevice default input" — it always exists.
        if current_device is None:
            current_still_present = True
        else:
            current_still_present = False
            for idx, name in new_candidates:
                if idx == current_device:
                    current_still_present = True
                    break
            # Also check against all PortAudio input devices (handles string names
            # and index mismatches after a PortAudio re-init).
            if not current_still_present:
                if isinstance(current_device, str):
                    for dev in devices:
                        if dev["max_input_channels"] == 0:
                            continue
                        if current_device in dev["name"].lower():
                            current_still_present = True
                            break
                else:
                    for dev in devices:
                        if dev["max_input_channels"] == 0:
                            continue
                        if dev["index"] == current_device:
                            current_still_present = True
                            break
 

        # If current device is gone, fall back to the first candidate.
        if not current_still_present and new_candidates:
            fallback_idx, fallback_name = new_candidates[0]
            new_probe = _ProbeDevice(fallback_idx, self._sample_rate, self._fft_size)
            new_probe.name = fallback_name
            new_probes.append(new_probe)
            new_candidates = [(idx, name) for idx, name in new_candidates if idx != fallback_idx]
            # Rebuild probe_map with the new current device.
            probe_map.clear()
            probe_map[fallback_idx] = new_probe
            for idx, name in new_candidates:
                probe = _ProbeDevice(idx, self._sample_rate, self._fft_size)
                probe.name = name
                probe_map[idx] = probe
            self._probes = [new_probe] + [probe_map[idx] for idx, _ in new_candidates]
            self._current_index = 0
            for idx, _ in new_candidates:
                new_device_indices.add(idx)
            # Signal the main loop to switch to the new current device.
            if self._switch_event is not None:
                self._switch_event.set()
            return True, new_device_indices

        # Start with the current probe.
        if self._probes:
            new_probes.append(self._probes[0])
        else:
            return False

        # Add or update candidate probes.
        old_candidates = self._probes[1:] if len(self._probes) > 1 else []
        old_candidate_map = {p.device: p for p in old_candidates}
        # Also exclude the current device so it's not counted as "new"
        known_indices = set(old_candidate_map.keys())
        if self._probes:
            known_indices.add(self._probes[0].device)

        for idx, name in new_candidates:
            if idx in known_indices:
                probe = old_candidate_map.get(idx)
                if probe is None:
                    # It's the current probe — reuse it at index 0 already
                    continue
                probe.name = name  # update name in case it changed
                new_probes.append(probe)
            else:
                # Genuinely new device — create a fresh probe.
                probe = _ProbeDevice(idx, self._sample_rate, self._fft_size)
                probe.name = name
                new_probes.append(probe)
                candidates_added = True
                new_device_indices.add(idx)
                probe_map[idx] = probe

        # Remove probes for devices that no longer exist.
        new_candidate_indices = {idx for idx, _ in new_candidates}
        for probe in old_candidates:
            if probe.device not in new_candidate_indices:
                if probe.device in probe_map:
                    del probe_map[probe.device]

        # Update the probe list (keeping current probe at index 0).
        self._probes = new_probes

        # Only signal device change when genuinely new devices are added
        # or existing devices are removed. Name-only updates don't count.
        devices_removed = any(
            p.device not in {idx for idx, _ in new_candidates}
            for p in old_candidates
        )
        return candidates_added or devices_removed, new_device_indices

    # -- the background worker thread -------------------------------------

    def _run(self) -> None:
        self._switch_event = threading.Event()
        self._running = True
        self._last_refresh = time.monotonic()
        print(f"  [monitor] refresh_interval={self._refresh_interval}s, probe_map={self._probe_map is not None}, probes={len(self._probes)}")

        while self._running:
            # Wait for interval
            self._switch_event.wait(self._interval)

            if not self._running:
                break

            # Periodically refresh the device list to pick up new/disconnected devices.
            now = time.monotonic()

            # Periodically refresh the device list to pick up new/disconnected devices.
            device_changed = False
            new_devices: set = set()
            if (self._probe_map is not None and
                    now - self._last_refresh >= self._refresh_interval):
                device_changed, new_devices = self._refresh_devices(self._probe_map)
                if device_changed:
                    print(f"  [refresh] Device list changed (changed={device_changed}, new={new_devices})")
                self._last_refresh = now

            if self._current_index >= len(self._probes):
                continue

            current_probe = self._probes[self._current_index]

            # --- persistence / locking ---
            # Only consider switching if the current device has been silent
            # for longer than the persistence timeout.  While the current
            # device is still active, we do NOT probe other devices.
            with current_probe._lock:
                current_silence = (now - current_probe._last_active
                                   if current_probe._last_active > 0 else float('inf'))
            can_switch = current_silence >= self._persistence_timeout

            current_db = self._rms_to_db(current_probe.energy)

            # When the current device is locked in (active within the persistence
            # window), back off probe frequency to persistence_timeout — there's
            # no point sampling other devices every 5 s when we won't switch anyway.
            probe_interval = self._interval if can_switch else self._persistence_timeout
            should_probe = device_changed or (now - self._last_probe >= probe_interval)

            if should_probe:
                self._last_probe = now
                if device_changed and new_devices:
                    print(f"  Device list changed (new={new_devices}), probing candidates...")
                else:
                    print(f"  Scanning for better audio device...")

                self._probe_all()

                best_idx = self._current_index
                best_db = current_db

                # Re-read current energy AFTER probing to get the live value.
                # The main callback may have updated it since we computed
                # current_db above.  Also, the probe's own _last_active
                # was set during probing if it saw energy > threshold.
                current_db = self._rms_to_db(current_probe.energy)

                for i, probe in enumerate(self._probes):
                    if i == self._current_index:
                        continue
                    p_db = self._rms_to_db(probe.energy)
                    print(f", {probe.name[:20]}={p_db:.1f} dB", end="")
                    if p_db > best_db and p_db > self._rms_to_db(self._threshold):
                        best_idx = i
                        best_db = p_db
                print()

                switch = False
                if device_changed and new_devices:
                    # Device list changed + new devices found.
                    # Only switch to the new device if it's a known-good monitor source
                    # (priority name match) OR if it already has audio signal.
                    # Avoid switching to raw ALSA hw devices with no signal — those
                    # are typically mic inputs that will never carry system audio.
                    _priority_pats = [
                        "monitor", "firefox", "chromium",
                        "easy effects", "easyeffects", "equalizer", "output level",
                    ]
                    best_new_probe = None
                    for probe in self._probes:
                        if probe.device not in new_devices:
                            continue
                        name_lower = probe.name.lower()
                        p_db = self._rms_to_db(probe.energy)
                        is_priority = any(p in name_lower for p in _priority_pats)
                        has_signal = p_db > self._rms_to_db(self._threshold)
                        if is_priority or has_signal:
                            best_new_probe = probe
                            break

                    if best_new_probe is not None:
                        current_name = current_probe.name
                        new_name = best_new_probe.name
                        print(f"  Auto-switch (new device): {current_name} -> {new_name}")
                        self._current_index = self._probes.index(best_new_probe)
                        self._switch_event.set()
                        switch = True
                    else:
                        print(f"  (new device(s) found but none are priority sources or have signal — keeping current)")
                elif best_idx != self._current_index:
                    margin_db = best_db - current_db
                    # Bypass hysteresis when the current device is effectively
                    # silent — there's nothing to "hold" onto so we should
                    # immediately switch to the first loud candidate.
                    current_silent = current_db < self._rms_to_db(self._threshold)
                    # Only allow margin-based switching when the current device
                    # has been silent long enough (can_switch).  This prevents
                    # switching away from a good active source just because
                    # another node briefly peaks louder (e.g. Spectrum catching
                    # the same audio, or a device-list refresh forcing a probe).
                    can_margin_switch = margin_db >= self._hysteresis_db and can_switch
                    can_silent_switch = current_silent and best_db > self._rms_to_db(self._threshold)
                    if can_margin_switch or can_silent_switch:
                        current_name = current_probe.name
                        new_name = self._probes[best_idx].name
                        print(f"  Auto-switch: {current_name} -> {new_name} "
                              f"({current_db:.1f} dB -> {best_db:.1f} dB)")
                        self._current_index = best_idx
                        self._switch_event.set()
                        switch = True
                    else:
                        print(f"    (no candidate loud enough by {self._hysteresis_db:.0f} dB; "
                              f"margin={margin_db:.1f} dB, best_idx={best_idx}, can_switch={can_switch})")

                # If ALL candidates are silent, speed up device-list refresh so a new
                # PipeWire monitor source (e.g. Firefox starting to play) is detected
                # within one probe cycle instead of waiting up to refresh_interval seconds.
                all_silent = (current_db < self._rms_to_db(self._threshold) and
                              best_db < self._rms_to_db(self._threshold))
                if all_silent:
                    self._refresh_interval = self._interval  # refresh every probe cycle
                else:
                    # Back off to the normal refresh cadence once we have signal.
                    self._refresh_interval = self._interval * 4

    # -- public API -------------------------------------------------------

    @property
    def interval(self) -> float:
        return self._interval

    @interval.setter
    def interval(self, value: float) -> None:
        self._interval = value

    @property
    def current_index(self) -> int:
        return self._current_index

    @current_index.setter
    def current_index(self, value: int) -> None:
        self._current_index = value

    def start(self) -> threading.Thread:
        self._interval = 5.0  # default interval in seconds
        self._current_index = 0
        self._last_probe = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="audio-monitor")
        self._thread.start()
        return self._thread

    def stop(self) -> None:
        self._running = False

    def wait_for_switch(self, timeout: float | None = None) -> bool:
        """Check (non-blocking when timeout=0) for a switch event.
        Returns True and clears the event if a switch was signalled.
        """
        if self._switch_event is None:
            return False
        return self._switch_event.wait(timeout=timeout)


# ---------------------------------------------------------------------------
# Main visualizer
# ---------------------------------------------------------------------------

class AudioVisualizer:
    def __init__(self, cfg: dict, auto_switch: bool = True, allow_mics: bool = False) -> None:
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

        # Auto-switch settings
        self._auto_switch: bool = auto_switch
        self._auto_switch_threshold: float = float(audio.get("auto_switch_threshold", -50))
        self._auto_switch_hysteresis: float = float(audio.get("auto_switch_hysteresis", 6))
        self._auto_switch_persistence: float = float(audio.get("auto_switch_persistence", 60))
        self._auto_switch_interval: float = float(audio.get("auto_switch_interval", 5))
        # CLI flag wins; config sets default
        self._allow_mics: bool = allow_mics or bool(audio.get("allow_mics", False))

        # Pre-computed state
        self._bass_skip: int = int(viz.get("bass_skip", 6))

        self._hann = np.hanning(self._fft_size).astype(np.float32)
        all_masks = make_log_bins(
            self._num_bars + self._bass_skip, self._freq_min, self._freq_max,
            self._fft_size, self._sample_rate,
        )
        self._rebuild_fft_bins(all_masks[self._bass_skip:])
        self._smooth_left  = np.zeros(self._num_bars, dtype=float)
        self._smooth_right = np.zeros(self._num_bars, dtype=float)
        self._peak_tracker_left  = PeakTracker(self._num_bars, peak_hold, peak_fall, peak_fade)
        self._peak_tracker_right = PeakTracker(self._num_bars, peak_hold, peak_fall, peak_fade)

        # Audio ring buffers — one per stereo channel (filled by callback, drained by main thread)
        # Implemented as a circular buffer; _ring_ptr is the next-write position.
        self._ring_left  = np.zeros(self._fft_size, dtype=np.float32)
        self._ring_right = np.zeros(self._fft_size, dtype=np.float32)
        self._ring_ptr   = 0
        self._audio_q: queue.Queue[tuple[np.ndarray, np.ndarray]] = queue.Queue(maxsize=8)

        # Current probe for auto-switch energy tracking (set before stream open)
        self._current_probe: Optional[_ProbeDevice] = None

        # Panels
        ext_map  = {"left": 0, "right": 1}   # bar extension direction
        freq_map = {"top": 0, "bottom": 2}    # low-freq position (bit 1)
        self._panels: list[Panel] = []
        self._left_panel:  Optional[Panel] = None
        self._right_panel: Optional[Panel] = None
        for side in ("left_panel", "right_panel"):
            pcfg = cfg.get(side, {})
            port = pcfg.get("port")
            if not port:
                continue
            flags  = ext_map.get(pcfg.get("direction", "left"), 0)
            flags |= freq_map.get(pcfg.get("freq_start", "top"), 0)
            if pcfg.get("mirror", False):
                flags ^= 0x01  # flip bar direction
            panel = Panel(port, flags, self._bar_fade_min)
            self._panels.append(panel)
            if side == "left_panel":
                self._left_panel = panel
            else:
                self._right_panel = panel

    # ------------------------------------------------------------------
    # FFT bin state helpers
    # ------------------------------------------------------------------

    def _rebuild_fft_bins(self, masks: list[np.ndarray]) -> None:
        """Pre-compute vectorized bin-to-band mapping from boolean masks.

        Stores a per-FFT-bin integer assignment array so _process can use
        np.bincount instead of iterating over 34 boolean masks per frame.
        """
        self._bin_masks = masks
        n_bins = self._fft_size // 2 + 1
        assign = np.full(n_bins, -1, dtype=np.int32)
        for i, mask in enumerate(masks):
            assign[mask] = i
        valid = assign >= 0
        counts = np.bincount(assign[valid], minlength=self._num_bars).astype(np.float32)
        self._bin_assign        = assign
        self._bin_assign_valid  = valid
        self._bin_assign_counts = counts

        # At low frequencies, log bands are narrower than one FFT bin, so multiple
        # bands snap to the same bin and only the last one wins in the assignment
        # above.  The earlier bands end up with count=0 and would show db_floor.
        # Fix: for each zero-count band, copy the value from the nearest live neighbor.
        zero_bands = np.where(counts == 0)[0]
        nz_bands   = np.where(counts > 0)[0]
        self._zero_band_sources: dict[int, int] = {}
        if len(zero_bands) and len(nz_bands):
            for z in zero_bands:
                nearest = nz_bands[int(np.argmin(np.abs(nz_bands - z)))]
                self._zero_band_sources[int(z)] = int(nearest)

    # ------------------------------------------------------------------
    # Audio callback — runs in a separate C thread; keep it minimal
    # ------------------------------------------------------------------

    def _audio_callback(self, indata: np.ndarray, frames: int, time_info, status) -> None:
        if indata.ndim > 1 and indata.shape[1] >= 2:
            left  = indata[:, 0].copy()
            right = indata[:, 1].copy()
        else:
            left = right = (indata[:, 0] if indata.ndim > 1 else indata.ravel()).copy()
        try:
            self._audio_q.put_nowait((left, right))
        except queue.Full:
            pass  # drop chunk — main loop is behind, visual glitch is acceptable

        # Track current device energy for auto-switch comparison (use L+R mean)
        if self._current_probe is not None:
            mono = (left + right) * 0.5
            energy = float(np.sqrt(np.mean(mono ** 2)))
            now = time.monotonic()
            with self._current_probe._lock:
                self._current_probe._energy = energy
                self._current_probe._updated = now
                if energy > 1e-6:
                    self._current_probe._last_active = now

    # ------------------------------------------------------------------
    # FFT + bar computation
    # ------------------------------------------------------------------

    def _process(self, samples: np.ndarray, smooth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return (bar_lengths, peak_lengths), each an int array clipped to [0, BAR_MAX].

        `smooth` is updated in-place — pass _smooth_left or _smooth_right.
        """
        windowed = samples * self._hann
        spectrum = np.abs(np.fft.rfft(windowed, n=self._fft_size))
        db = 20.0 * np.log10(spectrum + 1e-10)

        valid   = self._bin_assign_valid
        assigns = self._bin_assign[valid]
        sums    = np.bincount(assigns, weights=db[valid], minlength=self._num_bars)
        # np.where evaluates both branches, so guard against div-by-zero
        counts  = np.where(self._bin_assign_counts > 0, self._bin_assign_counts, 1)
        raw     = np.where(self._bin_assign_counts > 0,
                           sums / counts,
                           self._db_floor)
        for z, src in self._zero_band_sources.items():
            raw[z] = raw[src]

        # Map dB range to [0, 1]
        normalized = np.clip((raw - self._db_floor) / (self._db_ceil - self._db_floor), 0.0, 1.0)

        # Attack-fast / decay-slow smoothing
        rising = normalized > smooth
        smooth[rising] = (self._attack * normalized[rising]
                          + (1.0 - self._attack) * smooth[rising])
        smooth[~rising] *= self._decay

        # Bars use gamma curve; peaks stay linear so they always reach full brightness
        bars  = np.clip((smooth ** self._scale * BAR_MAX).astype(int), 0, BAR_MAX)
        peaks = np.clip((smooth * BAR_MAX).astype(int), 0, BAR_MAX)
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

    @staticmethod
    def _setup_pactl_monitor() -> Optional[str]:
        """Use pactl to find the default output sink's monitor source and set it
        as the PipeWire default source.  Returns the monitor source name or None.

        This ensures sounddevice's default input (device=None) routes to the
        output monitor, capturing all system audio regardless of which app
        is playing.
        """
        try:
            import subprocess
            r = subprocess.run(
                ["pactl", "get-default-sink"],
                capture_output=True, text=True, timeout=2,
            )
            if r.returncode != 0 or not r.stdout.strip():
                return None
            monitor = r.stdout.strip() + ".monitor"
            subprocess.run(
                ["pactl", "set-default-source", monitor],
                capture_output=True, text=True, timeout=2,
            )
            return monitor
        except Exception:
            return None

    def _find_device_monitor(self) -> Optional[object]:
        """Find a PipeWire tap source for system audio capture (Linux).

        Priority order:
        1. Named tap sources (Easy Effects, Equalizer, output-level meters, etc.)
           — these carry clean post-processed audio with no noise floor issues.
        2. pactl: set the PipeWire default source to the output-sink monitor and
           return None so sounddevice opens that default input.  Captures all
           system audio when no named tap source is present.
        3. Last resort: sounddevice default input (usually the output monitor on
           PipeWire desktops even without pactl).
        """
        devices = sd.query_devices()

        # --- priority scan: prefer named tap/post-processing sources ----------
        # These give a cleaner signal than the raw output monitor (no noise
        # floor bleed from other PipeWire nodes).  "easy effects source" is
        # the post-EQ output; prefer it over "easy effects sink" (pre-EQ).
        priority_names = [
            "easy effects source",
            "easyeffects source",
            "easy effects sink",
            "easyeffects sink",
            "equalizer",
            "output level meter",
            "monitor",
            "chromium",
        ]

        for pri in priority_names:
            for dev in devices:
                if dev["max_input_channels"] == 0:
                    continue
                if pri in dev["name"].lower():
                    print(f"  Auto-selected audio device: {dev['name']}")
                    return dev["index"]

        # --- pactl path: set the default source to the output monitor ----------
        monitor_name = self._setup_pactl_monitor()
        if monitor_name is not None:
            # Prefer a sounddevice device whose name contains "monitor" if one
            # is explicitly listed (some setups expose it as a named device).
            for dev in devices:
                if dev["max_input_channels"] == 0:
                    continue
                if "monitor" in dev["name"].lower() and dev["name"].lower() not in ("pipewire", "default"):
                    print(f"  Auto-selected output monitor: {dev['name']}")
                    return dev["index"]
            # No named monitor device — use the default input, which pactl just
            # pointed at the monitor.
            print(f"  Auto-selected output monitor (default source): {monitor_name}")
            return None

        # --- last-resort fallback ---------------------------------------------
        print("  Warning: no usable monitor source found — falling back to default input")
        print("  Tip: run --list-devices to see sounddevice inputs, or set audio.source in config.yaml")
        return None

    def _find_candidate_devices(self, exclude: object) -> list[tuple[object, str]]:
        """
        Return a list of (device, name) tuples for all usable input devices,
        excluding *exclude*.  Microphone devices are excluded by default;
        use --allow-mics to include them.

        The "default" sounddevice device is included when the initial device is
        not the default, because pactl may have configured it to point at the
        output monitor — making it the best fallback for system-audio capture.
        """
        candidates: list[tuple[object, str]] = []
        devices = sd.query_devices()
        default_included = False
        for dev in devices:
            if dev["index"] == exclude or dev["max_input_channels"] == 0:
                continue
            name_lower = dev["name"].lower()
            if name_lower == "pipewire":
                continue
            # Include "default" only once and only when the current device is not
            # the default (so we can auto-switch to the output monitor via pactl).
            if name_lower == "default":
                if exclude is not None and not default_included:
                    candidates.append((dev["index"], "default (output monitor)"))
                    default_included = True
                continue
            if not self._allow_mics and _is_microphone(dev["name"]):
                continue
            if _is_excluded_device(dev["name"]):
                continue
            candidates.append((dev["index"], dev["name"]))

        return candidates

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
                with _suppress_stderr():
                    sd.check_input_settings(device=device, channels=1, samplerate=wanted)
                return wanted
            except Exception:
                pass

        # Rebuild FFT state for the native rate
        print(f"  Sample rate {wanted} Hz not supported — using device native {native} Hz")
        self._sample_rate = native
        self._ring_left  = np.zeros(self._fft_size, dtype=np.float32)
        self._ring_right = np.zeros(self._fft_size, dtype=np.float32)
        self._ring_ptr   = 0
        all_masks = make_log_bins(
            self._num_bars + self._bass_skip, self._freq_min, self._freq_max,
            self._fft_size, native,
        )
        self._rebuild_fft_bins(all_masks[self._bass_skip:])
        self._hann = np.hanning(self._fft_size).astype(np.float32)
        return native

    # ------------------------------------------------------------------
    # Stream management
    # ------------------------------------------------------------------

    def _open_stream(self, device, sample_rate: int):
        """Open and start the main audio InputStream.

        Tries multiple sample rates and channel counts (stereo then mono)
        because PipeWire monitor sources may be stereo while hardware
        inputs are mono.

        Returns (stream, actual_rate).
        """
        # Suppress PortAudio/ALSA error messages at the C-level (fd 2).
        # PortAudio writes via vfprintf(stderr, ...) which bypasses Python's sys.stderr.
        with _suppress_stderr():
            extra = sd.WasapiSettings(loopback=True) if self._wasapi_loopback else None
            candidates = (sample_rate, 48000, 44100, 48001, 44101)
            for channels in (2, 1):
                for rate in candidates:
                    try:
                        stream = sd.InputStream(
                            device=device,
                            channels=channels,
                            samplerate=rate,
                            blocksize=256,
                            dtype=np.float32,
                            callback=self._audio_callback,
                            extra_settings=extra,
                        )
                        stream.start()
                        return stream, rate
                    except Exception:
                        pass
            # None of the rates worked — raise the last error
            stream = sd.InputStream(
                device=device,
                channels=1,
                samplerate=sample_rate,
                blocksize=256,
                dtype=np.float32,
                callback=self._audio_callback,
                extra_settings=extra,
            )
            stream.start()
            return stream, sample_rate

    def _switch_device(self, new_device, new_device_name: str) -> None:
        """Close current stream, resolve sample rate for new device, reopen stream."""
        # Close the old stream FIRST so ALSA releases the device handle.
        if self._stream is not None:
            try:
                self._stream.close()
            except Exception:
                pass
        sample_rate = self._resolve_sample_rate(new_device)
        self._ring_left  = np.zeros(self._fft_size, dtype=np.float32)
        self._ring_right = np.zeros(self._fft_size, dtype=np.float32)
        self._ring_ptr   = 0
        self._smooth_left[:]  = 0.0
        self._smooth_right[:] = 0.0
        stream, actual_rate = self._open_stream(new_device, sample_rate)
        self._stream = stream
        if actual_rate != sample_rate:
            print(f"  Sample rate {sample_rate} Hz not supported — using {actual_rate} Hz")
            self._sample_rate = actual_rate
            all_masks = make_log_bins(
                self._num_bars + self._bass_skip, self._freq_min, self._freq_max,
                self._fft_size, actual_rate,
            )
            self._rebuild_fft_bins(all_masks[self._bass_skip:])

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        if not self._panels:
            print("ERROR: no panels configured. Set left_panel.port / right_panel.port in config.yaml")
            sys.exit(1)

        print("Connecting to panels...")
        for panel in self._panels:
            try:
                panel.connect()
            except serial.SerialException as e:
                port_name = panel.port if hasattr(panel, "port") else "unknown"
                print(f"ERROR: could not open panel port {port_name}: {e}")
                # Show available serial devices
                available = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
                if available:
                    print(f"Available serial devices: {', '.join(available)}")
                else:
                    print("No serial devices found. Check that panels are connected and recognized by the system.")
                print("Update the port in config.yaml or pass --left / --right on the command line.")
                sys.exit(1)

        device = self._find_device()
        sample_rate = self._resolve_sample_rate(device)
        frame_interval = 1.0 / self._target_fps

        # Build probes FIRST so the callback can track energy from the start
        current_probe: Optional[_ProbeDevice] = None
        monitor: Optional[AudioMonitor] = None
        monitor_thread: Optional[threading.Thread] = None
        probe_map: dict[object, _ProbeDevice] = {}

        if self._auto_switch:
            candidates = self._find_candidate_devices(device)
            if candidates:
                current_probe = _ProbeDevice(device, self._sample_rate, self._fft_size)
                try:
                    info = sd.query_devices(device, kind="input")
                    current_probe.name = info["name"]
                except Exception:
                    current_probe.name = str(device)
                probe_map[device] = current_probe

                for dev_idx, dev_name in candidates:
                    probe = _ProbeDevice(dev_idx, self._sample_rate, self._fft_size)
                    probe.name = dev_name
                    probe_map[dev_idx] = probe

                probes_list = [current_probe] + [probe_map[d] for d, _ in candidates]

                monitor = AudioMonitor(
                    probe_devices=probes_list,
                    current_device=device,
                    sample_rate=self._sample_rate,
                    threshold=self._auto_switch_threshold,
                    hysteresis_db=self._auto_switch_hysteresis,
                    persistence_timeout=self._auto_switch_persistence,
                    refresh_interval=self._auto_switch_interval * 4,
                    allow_mics=self._allow_mics,
                    probe_map=probe_map,
                    fft_size=self._fft_size,
                )
                monitor.interval = self._auto_switch_interval
                monitor.current_index = 0
                # Set name on the current probe
                if current_probe:
                    current_probe.name = probe_map[device].name
                monitor_thread = monitor.start()
                print(f"  Auto-switch enabled — scanning {len(probes_list)} devices "
                      f"every {self._auto_switch_interval:.1f}s")

        # Assign current probe BEFORE opening the stream so the callback
        # sees a non-None _current_probe on its very first invocation.
        self._current_probe = current_probe

        # Open the initial audio stream (must happen before main loop)
        stream, actual_rate = self._open_stream(device, sample_rate)
        self._stream = stream
        if actual_rate != sample_rate:
            print(f"  Sample rate {sample_rate} Hz not supported — using {actual_rate} Hz")
            self._sample_rate = actual_rate
            all_masks = make_log_bins(
                self._num_bars + self._bass_skip, self._freq_min, self._freq_max,
                self._fft_size, actual_rate,
            )
            self._rebuild_fft_bins(all_masks[self._bass_skip:])

        last_frame = time.monotonic()

        try:
            while True:
                # Check for auto-switch signal (non-blocking)
                # wait_for_switch returns True only when a new switch is signalled
                # and clears the event, so repeated calls won't re-trigger.
                if monitor is not None:
                    if monitor.wait_for_switch(timeout=0):
                        # A switch was signalled — clear the event so we don't
                        # re-trigger on every loop iteration.
                        monitor._switch_event.clear()
                        # The monitor already updated current_index.
                        # Look up device from the monitor's probe list (which may
                        # have been updated by _refresh_devices).
                        new_probe = monitor._probes[monitor.current_index]
                        new_device = new_probe.device
                        new_name = new_probe.name
                        self._stream.stop()
                        self._switch_device(new_device, new_name)
                        # Sync the main-loop probe pointer and start the persistence
                        # lock from this moment so the monitor doesn't re-probe.
                        self._current_probe = probe_map[new_device]
                        self._current_probe._last_active = time.monotonic()
                        print("  Resume streaming...")

                # Accumulate audio into per-channel ring buffers (circular, no roll)
                try:
                    left_chunk, right_chunk = self._audio_q.get(timeout=0.2)
                except queue.Empty:
                    continue

                n   = len(left_chunk)
                ptr = self._ring_ptr
                end = ptr + n
                if end <= self._fft_size:
                    self._ring_left[ptr:end]  = left_chunk
                    self._ring_right[ptr:end] = right_chunk
                else:
                    split = self._fft_size - ptr
                    self._ring_left[ptr:]      = left_chunk[:split]
                    self._ring_left[:n - split] = left_chunk[split:]
                    self._ring_right[ptr:]      = right_chunk[:split]
                    self._ring_right[:n - split] = right_chunk[split:]
                self._ring_ptr = end % self._fft_size

                now = time.monotonic()
                if now - last_frame < frame_interval:
                    continue
                last_frame = now

                # Unroll circular buffer into a contiguous window for FFT
                ptr = self._ring_ptr
                if ptr == 0:
                    left_window  = self._ring_left
                    right_window = self._ring_right
                else:
                    left_window  = np.concatenate((self._ring_left[ptr:],  self._ring_left[:ptr]))
                    right_window = np.concatenate((self._ring_right[ptr:], self._ring_right[:ptr]))

                bars_l, peaks_l = self._process(left_window,  self._smooth_left)
                bars_r, peaks_r = self._process(right_window, self._smooth_right)

                def _build_eq(bars, peaks, tracker):
                    pad = NUM_BANDS - self._num_bars
                    h = bars.tolist() + [0] * pad
                    if self._peaks_enabled:
                        pa, pb = tracker.update(peaks)
                        p = pa.tolist() + [0] * pad
                        b = pb.tolist() + [0] * pad
                    else:
                        p = [0] * NUM_BANDS
                        b = [0] * NUM_BANDS
                    return h, p, b

                h_l, p_l, b_l = _build_eq(bars_l, peaks_l, self._peak_tracker_left)
                h_r, p_r, b_r = _build_eq(bars_r, peaks_r, self._peak_tracker_right)

                if self._left_panel:
                    self._left_panel.send_eq(h_l, p_l, b_l)
                if self._right_panel:
                    self._right_panel.send_eq(h_r, p_r, b_r)

        except KeyboardInterrupt:
            print("\nStopping...")
            zeros = [0] * NUM_BANDS
            for panel in self._panels:
                panel.send_eq(zeros, zeros, zeros)
        finally:
            if self._stream is not None:
                try:
                    self._stream.stop()
                    self._stream.close()
                except Exception:
                    pass
            for panel in self._panels:
                panel.close()
            # Stop the monitor thread
            if monitor is not None:
                monitor.stop()


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
    p.add_argument("--no-auto-switch", action="store_true",
                   help="Disable auto-switching to the loudest audio source")
    p.add_argument("--allow-mics", action="store_true",
                   help="Include microphone devices in auto-switch candidate list")
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
    if args.allow_mics:
        cfg.setdefault("audio", {})["allow_mics"] = True

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

    AudioVisualizer(cfg, auto_switch=not args.no_auto_switch, allow_mics=args.allow_mics).run()


if __name__ == "__main__":
    main()
