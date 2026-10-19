import unittest

from raton_sim.giros import native as N
from raton_sim.seguridad import orchestrators as O
from raton_sim.simulacion import simulator as S


class OrchestratorTests(unittest.TestCase):
    def test_ghost_cannot_dominate_median(self):
        self.assertFalse(O.wall_classifier([35, 180, 185], 100))
        self.assertFalse(O.wall_classifier([35], 100))
        self.assertTrue(O.wall_classifier([35, 40, 180], 100))

    def test_front_requires_independent_consecutive_close_samples(self):
        m = O.MotionOrchestrator()
        self.assertFalse(m.confirmed_front(65, 1))
        self.assertFalse(m.confirmed_front(65, 1))
        self.assertFalse(m.confirmed_front(200, 2))
        self.assertFalse(m.confirmed_front(65, 3))
        self.assertTrue(m.confirmed_front(70, 4))
        self.assertFalse(m.confirmed_front(-1, 5))

    def test_stale_wait_requires_new_deliveries_without_fixed_delay(self):
        s = S.Sim(S.Case("fresh", S.M_L, (2, 0), (0, 2), fw=N.firmware_config(S)))
        s.readings["F"] = 45
        list(s.wait_fresh_front())
        self.assertGreaterEqual(s.nsamples, 2)
        self.assertLess(s.t, 0.1)
        self.assertEqual(s.readings["F"], s.raw_readings["F"])

    def test_start_wall_chooses_and_verifies_side(self):
        class Start(S.Sim):
            turn_log = []

            def wait_fresh_front(self):
                self.raw_readings["F"] = 80
                yield from ()

            def align_front(self):
                self.aligned = True
                yield from ()

            def turn_corner_track(self, degrees):
                self.turn_log = self.turn_log + [degrees]
                self.raw_readings["F"] = 250
                yield from ()

            def explore(self, pos, heading, prior=None):
                self.chosen_heading = heading
                yield from ()

        s = Start(S.Case("start-wall", S.M_L, (2, 0), (0, 2), mode=3))
        s.readings.update(L=80, R=180)
        list(s.mode3())
        self.assertTrue(s.aligned)
        self.assertEqual(s.turn_log, [-90])
        self.assertEqual(s.chosen_heading, 1)

    def test_front_alignment_uses_nominal_distance_both_directions(self):
        class Align(S.Sim):
            def tick(self):
                super().tick()
                self.readings["F"] = self.front_turn() + self.offset - self.travelled

        for offset in (-20, 20):
            s = Align(S.Case("offset", S.M_L, (2, 0), (0, 2), mode=3, corridor=192))
            s.offset = offset
            list(s.align_front())
            self.assertAlmostEqual(s.travelled, offset, delta=3)
            self.assertAlmostEqual(s.pending, 0, delta=3)

    def test_invalid_gate_falls_back(self):
        self.assertEqual(O.Gate([[float("nan")]]).weights_for([1], 1), [1])
        self.assertEqual(O.Gate(None).weights_for([1], 2), [1, 1])

    def test_failure_policy_marks_unreachable_and_resets(self):
        m = O.MissionOrchestrator()
        self.assertTrue(m.progress(False, (0, 0), 2))
        self.assertFalse(m.progress(False, (0, 0), 2))
        self.assertIn((0, 0), m.unreachable)
        self.assertTrue(m.progress(True, (1, 0), 2))

    def test_safety_veto_cannot_be_removed_by_gate(self):
        self.assertEqual(O.MotionOrchestrator([[100]]).safety(0, 0.1, 3, 3), "veto")


if __name__ == "__main__":
    unittest.main()
