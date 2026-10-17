"""Domain benchmark invariants: seeded scenarios and conservative learned routes."""

import unittest

from raton_sim.training import benchmark as B


class RobustTests(unittest.TestCase):
    def test_domains_are_reproducible_and_bounded(self):
        for w in B.WIDTHS:
            a, maze = B.scenario(w, 123, "combined")
            self.assertEqual((a, maze), B.scenario(w, 123, "combined"))
            self.assertTrue(23 <= a["angle"] <= 38)
            self.assertTrue(10 <= a["wall"] <= 18)
            self.assertTrue(abs(a["off"]) <= 40)
            self.assertIsNotNone(B.path_length(maze, a["start"], a["goal"]))

    def test_unknown_edges_cannot_form_a_learned_route(self):
        d, m = B.scenario(192, 123, "nominal")
        sim = B.RandomizedSim(B.S.Case("unknown", m, tuple(d["start"]), tuple(d["goal"])), d, 1)
        self.assertIsNone(B.path_length(B.learned_maze(sim), sim.c.start, sim.c.goal))

    def test_encoder_loss_does_not_shrink_physical_motion(self):
        d, m = B.scenario(192, 123, "nominal")
        d["loss"] = 0.05
        sim = B.RandomizedSim(B.S.Case("enc", m, tuple(d["start"]), tuple(d["goal"])), d, 1)
        y = sim.y
        nav = sim.nav_y
        self.assertTrue(sim.try_move(sim.x, sim.y + 1, sim.th))
        self.assertAlmostEqual(sim.y - y, 1)
        self.assertAlmostEqual(sim.nav_y - nav, 0.95)

    def test_dead_sensor_is_invalid_after_filter(self):
        d, m = B.scenario(192, 123, "dead_right")
        sim = B.RandomizedSim(B.S.Case("dead", m, tuple(d["start"]), tuple(d["goal"])), d, 1)
        for _ in range(5):
            sim.sample()
        self.assertEqual(sim.readings["R"], -1)


if __name__ == "__main__":
    unittest.main()
