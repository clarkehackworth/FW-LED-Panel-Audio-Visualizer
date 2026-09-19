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

if __name__ == "__main__":
    test(); print("ok")
