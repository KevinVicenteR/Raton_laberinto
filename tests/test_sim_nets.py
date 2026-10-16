"""Redes de control y de logica: mismas entradas e inferencia en Python (tools/nets.py) y C++ (firmware)."""
import ctypes
import json
import math
import os
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import nets  # noqa: E402

FP = ctypes.POINTER(ctypes.c_float)


def native():
    path = os.environ.get('RATON_NATIVE_LIB') or next(
        (str(p) for p in (ROOT / 'build/libraton_navigation.dylib', ROOT / 'build/libraton_navigation.so') if p.exists()), None)
    if not path:
        raise unittest.SkipTest('falta la biblioteca nativa raton_navigation')
    lib = ctypes.CDLL(path)
    lib.raton_control_features.argtypes = [FP, *([ctypes.c_float] * 5), ctypes.c_int, FP]
    lib.raton_logic_features.argtypes = [FP, ctypes.c_int, *([ctypes.c_float] * 3), ctypes.c_int, FP]
    lib.raton_control_forward.argtypes = [FP, FP]
    lib.raton_logic_forward.argtypes = [FP, FP]
    return lib


def arr(values, n=None):
    return (ctypes.c_float * (n or len(values)))(*values)


class NetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = native()
        cls.rng = random.Random(5)

    def test_control_features_match(self):
        for _ in range(200):
            r = self.rng
            history = [[(r.uniform(-50, 400), r.random() < .8, r.random(), r.uniform(0, 1500)) for _ in range(3)]
                       for _ in range(3)]
            args = (r.uniform(-50, 400), r.uniform(-300, 300), r.uniform(-60, 60), r.uniform(-300, 900),
                    r.uniform(-6, 6), r.random() < .3)
            py = nets.control_vector(history, *args)
            flat = [v for frame in history for (mm, ok, c, a) in frame for v in (mm, float(ok), c, a)]
            out = arr([], nets.CTRL_IN)
            self.lib.raton_control_features(arr(flat), *args[:5], int(args[5]), out)
            for a, b in zip(py, out):
                self.assertAlmostEqual(a, b, places=5)

    def test_logic_features_match(self):
        for _ in range(200):
            r = self.rng
            dirs = [(r.random() < .6, r.random() < .5, r.random() < .5, r.random() < .4, r.choice((999, r.randrange(80))),
                     r.randrange(9), r.uniform(-1, 1)) for _ in range(4)]
            here, seen, fwd, right, heading = r.randrange(100), r.random(), r.uniform(-1, 1), r.uniform(-1, 1), r.randrange(4)
            py = nets.logic_vector(dirs, here, seen, fwd, right, heading)
            flat = [float(v) for d in dirs for v in d]
            out = arr([], nets.LOGIC_IN)
            self.lib.raton_logic_features(arr(flat), here, seen, fwd, right, heading, out)
            for a, b in zip(py, out):
                self.assertAlmostEqual(a, b, places=5)

    def check_model(self, kind, n_in, forward):
        path = ROOT / f'tools/models/{kind}_model.json'
        header = (ROOT / f'lib/raton_core/src/{kind}_model.h').read_text()
        if not path.exists() or 'Trained = true' not in header:
            self.skipTest(f'red de {kind} sin entrenar')
        net = nets.Mlp(json.loads(path.read_text()))
        for _ in range(100):
            x = [self.rng.uniform(-1, 1.5) for _ in range(n_in)]
            out = arr([], net.m['architecture'][-1])
            self.assertEqual(forward(arr(x), out), 1)
            for a, b in zip(net(x), out):
                self.assertLess(abs(a - b), 2e-4 * max(1, abs(a)))
        self.assertIsNone(net([float('nan')] * n_in))

    def test_control_inference_matches_firmware(self):
        self.check_model('control', nets.CTRL_IN, self.lib.raton_control_forward)

    def test_logic_inference_matches_firmware(self):
        self.check_model('logic', nets.LOGIC_IN, self.lib.raton_logic_forward)

    def test_logic_only_picks_allowed(self):
        for _ in range(500):
            heading = self.rng.randrange(4)
            opts = self.rng.sample(range(4), self.rng.randint(1, 4))
            scores = [self.rng.uniform(-5, 5) for _ in range(4)]
            self.assertIn(nets.logic_pick(scores, heading, opts), opts)

    def test_control_outputs_bounded(self):
        for v in (-1e6, -3, 0, 3, 1e6):
            dyaw, scale = nets.control_apply([v, v])
            self.assertLessEqual(abs(dyaw), nets.CTRL_YAW_MAX + 1e-9)
            self.assertTrue(nets.CTRL_VMIN - 1e-9 <= scale <= 1 + 1e-9)
        self.assertEqual(nets.control_apply(None), (0.0, 1.0))

    def test_maze_splits_disjoint(self):
        self.assertFalse(set(nets.TRAIN_MAZES) & set(nets.VAL_MAZES))
        self.assertFalse((set(nets.TRAIN_MAZES) | set(nets.VAL_MAZES)) & nets.TEST_MAZES)


if __name__ == '__main__':
    unittest.main()
