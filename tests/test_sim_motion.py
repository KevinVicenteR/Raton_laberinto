"""Regresiones de movimiento: python3 -m unittest discover -s tests -p 'test_sim_*.py'."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import sim_pistas as P


class MotionTests(unittest.TestCase):
    def make_sim(self, cls=P.S.Sim):
        return cls(P.S.Case('regresion', P.S.comp_maze(), P.START, P.GOAL,
                            mode=4, fw=P.FW, seconds=120), seed=1)

    def test_blocked_rotation_reports_failure(self):
        class Blocked(P.S.Sim):
            def try_move(self, *args):
                return False
            def mode4(self):
                yield from self.rotate(90)
                self.continued = True
        sim = self.make_sim(Blocked).run()
        self.assertEqual(sim.failure, 'giro incompleto')
        self.assertFalse(hasattr(sim, 'continued'))
        self.assertIsNone(sim.reached)

    def test_corner_accounts_for_actual_diagonal_distance(self):
        class ShortCorner(P.S.Sim):
            def rotate(self, deg):
                yield from ()
            def drive(self, mm, watch, **kwargs):
                self.travelled += 25  # el movimiento se interrumpe antes de los 75 mm pedidos
                yield from ()
        sim = self.make_sim(ShortCorner)
        list(sim.turn_corner_track(90))
        self.assertAlmostEqual(sim.pending, 25 * math.sin(math.radians(70)) + 33)

    def test_free_navigation_diagonal_uses_corner_protection(self):
        class Corner(P.S.Sim):
            def rotate(self, deg):
                yield from ()
            def drive(self, mm, watch, **kwargs):
                self.protected = kwargs.get('corner', False)
                yield from ()
        sim = self.make_sim(Corner)
        list(sim.safe_turn(90))
        self.assertTrue(sim.protected)


if __name__ == '__main__':
    unittest.main()
