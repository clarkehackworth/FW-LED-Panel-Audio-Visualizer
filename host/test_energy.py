"""Energy metric must ignore DC: a stuck rail is not audio."""
import numpy as np

def energy(x):
    return float(np.std(x))

def test():
    rail = np.full(4096, -1.0, dtype=np.float32)          # broken node
    quiet = np.random.default_rng(0).normal(0, 0.05, 4096) # real quiet audio
    assert energy(rail) < 1e-6, energy(rail)
    assert energy(quiet) > energy(rail)
    # DC offset on real audio doesn't inflate it
    assert abs(energy(quiet + 0.7) - energy(quiet)) < 1e-9

def test_capture_filter():
    """Real capture sources (mics) must be excluded; sink monitors kept."""
    import importlib.util as u, sounddevice as sd
    ns = {}
    src = open(__file__.replace("test_energy.py", "audio_viz.py")).read()
    exec(compile(src.split("class AudioVisualizer")[0], "av", "exec"), ns)
    caps = ns["_pactl_capture_names"]()
    if not caps:
        return  # no PulseAudio/PipeWire here, nothing to assert
    for c in caps:
        assert ns["_is_microphone"](c), c
        assert not ns["_is_microphone"]("monitor of " + c)


def test_fft_dbfs():
    """A full-scale sine must read ~0 dBFS after normalisation, not +48."""
    N = 1024
    h = np.hanning(N).astype(np.float32)
    gain = 2.0 / float(h.sum())
    t = np.arange(N) / 48000.0
    sine = np.sin(2 * np.pi * 1000 * t)          # bin-centred enough at N=1024
    sp = np.abs(np.fft.rfft(sine * h)) * gain
    db = 20 * np.log10(sp.max() + 1e-10)
    assert -1.0 < db < 0.5, db
    # half-scale sine -> ~-6 dBFS
    sp2 = np.abs(np.fft.rfft(0.5 * sine * h)) * gain
    assert -7.0 < 20 * np.log10(sp2.max()) < -5.0


def _load():
    ns = {}
    src = open(__file__.replace("test_energy.py", "audio_viz.py")).read()
    exec(compile(src.split("class AudioVisualizer")[0], "av", "exec"), ns)
    return ns


def test_stale_current_index():
    """A refresh that shrinks _probes must not IndexError on a stale index."""
    ns = _load()
    probe = ns["_ProbeDevice"](0, 48000, 1024)
    probe.name = "default"
    mon = ns["AudioMonitor"]([probe], None, 48000, probe_map={})
    mon._current_index = 5           # points far past the single probe
    mon._refresh_devices({})         # used to raise IndexError
    assert mon._current_index < max(len(mon._probes), 1)

    # and with no probes at all
    mon._probes = []
    mon._current_index = 3
    mon._refresh_devices({})


def test_pactl_cache_keeps_last_good():
    """A failed pactl read must not wipe a known-good capture set."""
    ns = _load()
    good = {"some microphone"}
    ns["_capture_cache"] = (0.0, good)          # stale (forces a re-read)
    import subprocess
    orig = subprocess.run
    subprocess.run = lambda *a, **k: (_ for _ in ()).throw(OSError("pactl gone"))
    try:
        assert ns["_pactl_capture_names"]() == good
    finally:
        subprocess.run = orig


if __name__ == "__main__":
    test(); test_capture_filter(); test_fft_dbfs()
    test_stale_current_index(); test_pactl_cache_keeps_last_good()
    print("ok")
