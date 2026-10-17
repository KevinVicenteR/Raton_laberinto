"""Regresiones de movimiento: python3 -m unittest discover -s tests -p 'test_sim_*.py'."""

import math
import unittest

from raton_sim import tracks as P


class MotionTests(unittest.TestCase):
    def make_sim(self, cls=P.S.Sim):
        return cls(
            P.S.Case("regresion", P.S.comp_maze(), P.START, P.GOAL, mode=4, fw=P.FW, seconds=120), seed=1
        )

    def test_finite_drive_completes_and_records_distance(self):
        sim = self.make_sim()
        list(sim.drive(2, 0))
        self.assertGreaterEqual(sim.travelled, 1)
        self.assertTrue(sim.motions[-1]["completed"])
        self.assertEqual(sim.motions[-1]["reason"], "complete")

    def test_blocked_rotation_reports_failure(self):
        class Blocked(P.S.Sim):
            def try_move(self, *args):
                return False

            def mode4(self):
                yield from self.rotate(90)
                self.continued = True

        sim = self.make_sim(Blocked).run()
        self.assertEqual(sim.failure, "giro incompleto")
        self.assertFalse(hasattr(sim, "continued"))
        self.assertIsNone(sim.reached)

    def test_corner_rotates_in_place_and_clears_pending(self):
        sim = self.make_sim()
        sim.pending = 12
        start = sim.x, sim.y
        target = sim.th + math.pi / 2
        list(sim.turn_corner_track(90))
        self.assertEqual((sim.x, sim.y), start)
        self.assertEqual(sim.pending, 0)
        self.assertLessEqual(abs(sim.th - target), math.radians(0.5))

    def test_real_geometry_and_legacy_preset(self):
        robot = P.S.Robot()
        self.assertEqual((robot.length_front, robot.length_back, robot.width), (40, 40, 80))
        sim = self.make_sim()
        self.assertAlmostEqual(sim.side_face(40 / math.sin(math.radians(31))), 30)
        self.assertEqual(P.S.Robot.preset("antiguo").length_front, 83)

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
        self.assertLessEqual(abs(sim.th - target), math.radians(0.5))
        self.assertGreater(sim.t, 0.1)  # correction consumes actual simulated time

    def test_blocked_precision_stage_does_not_confirm_turn(self):
        class BlockedFine(P.S.Sim):
            def rotate(self, degrees):
                yield from ()

            def try_move(self, *args):
                return False

        sim = self.make_sim(BlockedFine)
        with self.assertRaisesRegex(P.S.MotionFailure, "refinamiento angular incompleto"):
            list(sim.rotate_precise_to(sim.th + math.radians(6)))
        self.assertLessEqual(sim.t, 1.01)

    def test_goal_memory_strategy_is_explicitly_opt_in(self):
        from dataclasses import replace

        class Remember(P.S.Sim):
            def explore(self, start, heading, prior=None):
                self.used_memory = (start, heading)
                yield from ()

        sim = self.make_sim(Remember)
        self.assertFalse(P.S.Firmware().goal_grid)
        sim.c = replace(sim.c, use_goal=True, fw=replace(sim.c.fw, goal_grid=True))
        list(sim.mode5())
        self.assertEqual(sim.used_memory, (sim.c.start, 0))


if __name__ == "__main__":
    unittest.main()
