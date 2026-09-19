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


if __name__ == "__main__":
    test(); test_capture_filter(); print("ok")
