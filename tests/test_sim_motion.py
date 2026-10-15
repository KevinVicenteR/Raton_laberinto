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


    def test_precision_feedback_corrects_coarse_endpoint_error(self):
        class CoarseError(P.S.Sim):
            def rotate(self, degrees):
                self.th += math.radians(degrees - 6)
                yield from ()
            def try_move(self, x, y, heading):
                self.th = heading
                return True
        sim = self.make_sim(CoarseError)
        target = sim.th + math.radians(70)
        list(sim.rotate_precise_to(target))
        self.assertLessEqual(abs(sim.th-target), math.radians(.5))
        self.assertGreater(sim.t, .1)  # correction consumes actual simulated time

    def test_blocked_precision_stage_does_not_confirm_turn(self):
        class BlockedFine(P.S.Sim):
            def rotate(self, degrees):
                yield from ()
            def try_move(self, *args):
                return False
        sim = self.make_sim(BlockedFine)
        with self.assertRaisesRegex(P.S.MotionFailure, 'refinamiento angular incompleto'):
            list(sim.rotate_precise_to(sim.th+math.radians(6)))
        self.assertLessEqual(sim.t, 1.01)

    def test_goal_memory_strategy_is_explicitly_opt_in(self):
        from dataclasses import replace
        class Remember(P.S.Sim):
            def explore(self, start, heading, prior=None):
                self.used_memory = (start, heading)
                yield from ()
        sim = self.make_sim(Remember)
        self.assertFalse(sim.c.fw.goal_grid)
        sim.c = replace(sim.c, use_goal=True, fw=replace(sim.c.fw, goal_grid=True))
        list(sim.mode5())
        self.assertEqual(sim.used_memory, (sim.c.start, 0))


if __name__ == '__main__':
    unittest.main()
