"""Measured pitch and frontier exploration regressions."""

import unittest

from raton_sim import simulator as S


class CellTests(unittest.TestCase):
    def test_measure_supported_widths_without_truth_input(self):
        class Measured(S.Sim):
            def turn_corner_track(self, degrees):
                self.direction = getattr(self, "direction", 0) + degrees
                yield from ()

            def tick(self):
                self.t += self.DT
                self.readings["F"] = self.fake if self.direction else -1

        for width in (140, 180, 220, 260):
            sim = Measured(S.Case("measure", S.M_L, (2, 0), (0, 2), corridor=999))
            sim.fake = width / 2 - 40 + 50
            list(sim.measure_cell())
            self.assertEqual(sim.cell_mm, width + 12)
            import math

            self.assertAlmostEqual(
                sim.c.fw.side_look_ahead, 30 + (width / 2 - 30) / math.tan(math.radians(31))
            )
            self.assertEqual(sim.c.fw.cell_side_wall, max(45, int(0.42 * width)))
            self.assertEqual(sim.c.fw.corner_front, max(35, width / 2 - 40))

    def test_configured_width_sets_cell_sampling_geometry(self):
        import math

        for width in (140, 168, 192, 220):
            sim = S.Sim(S.Case("geometry", S.M_L, (2, 0), (0, 2), corridor=width))
            self.assertAlmostEqual(
                sim.c.fw.side_look_ahead, 30 + (width / 2 - 30) / math.tan(math.radians(31))
            )
            # Mismo umbral desde el costado que TrackGeometry::cellSideWall.
            self.assertEqual(sim.c.fw.cell_side_wall, max(45, int(0.42 * width)))

    def test_measure_missing_wall_keeps_default(self):
        class Missing(S.Sim):
            def turn_corner_track(self, degrees):
                yield from ()

            def tick(self):
                self.t += self.DT

        sim = Missing(S.Case("missing", S.M_L, (2, 0), (0, 2)))
        list(sim.measure_cell())
        self.assertEqual(sim.cell_mm, 155)

    def test_full_mapping_does_not_stop_on_goal(self):
        class Mapping(S.Sim):
            def mode3(self):
                self.reached = 0.01
                for _ in range(10):
                    self.t += self.DT
                    yield
                self.mapping_complete = True

        sim = Mapping(S.Case("full", S.M_L, (2, 0), (2, 0), mode=3)).run()
        self.assertTrue(sim.mapping_complete)
        self.assertIsNone(sim.failure)

    def test_turn_retries_stopped_rotation(self):
        import math

        class Retry(S.Sim):
            calls = 0

            def rotate_precise_to(self, target):
                self.calls += 1
                if self.calls == 1:
                    self.th = target - math.radians(7)
                    raise S.MotionFailure("stopped")
                self.th = target
                yield from ()

        sim = Retry(S.Case("retry", S.M_L, (2, 0), (0, 2)))
        list(sim.cell_rotate_to(sim.th + math.pi / 2))
        self.assertEqual(sim.calls, 2)

    def test_persistent_scrape_backs_twice_then_reports_stuck(self):
        class Scrape(S.Sim):
            backs = 0

            def tick(self):
                self.t += self.DT
                self.readings = dict(L=20, F=-1, R=-1)

            def try_move(self, x, y, heading):
                return True

            def back(self, mm):
                self.backs += 1
                self.travelled -= mm
                yield from ()

            def cell_rotate_to(self, target):
                yield from ()

        sim = Scrape(S.Case("scrape", S.M_L, (2, 0), (0, 2)))
        with self.assertRaisesRegex(S.MotionFailure, "PEGADO"):
            list(sim.run_cells(1, 100))
        self.assertEqual(sim.backs, 2)
        self.assertEqual(sim.recoveries, 3)

    def test_cell_run_accepts_equivalent_grid_angles(self):
        import math

        class Straight(S.Sim):
            def tick(self):
                self.t += self.DT

            def try_move(self, x, y, heading):
                self.travelled += math.hypot(x - self.x, y - self.y)
                self.x, self.y, self.th = x, y, heading
                return True

        sim = Straight(S.Case("wrapped", S.M_L, (2, 0), (0, 2), mode=3))
        initial = sim.th
        sim.grid_th = initial + 2 * math.pi
        list(sim.run_cells(1, 100))
        self.assertAlmostEqual(sim.th, initial)
        self.assertEqual(sim.recoveries, 0)


if __name__ == "__main__":
    unittest.main()
