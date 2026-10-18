"""Regresiones de montajes diagonales, anticipacion y avance nulo."""

import math
import unittest

from raton_sim import simulator as S


class GeometryTests(unittest.TestCase):
    def test_sensor_mounts_and_directions(self):
        sim = S.Sim(S.Case("mounts", S.M_L, (2, 0), (0, 2)))
        calls = []
        sim.raycast = lambda x, y, angle: calls.append((x, y, angle))
        sim.sample()
        front, left, right = calls
        self.assertAlmostEqual(front[0], sim.x)
        self.assertAlmostEqual(front[1], sim.y + 40)
        self.assertAlmostEqual(left[0], sim.x - 30)
        self.assertAlmostEqual(right[0], sim.x + 30)
        self.assertAlmostEqual(left[1], sim.y + 30)
        self.assertAlmostEqual(right[1], sim.y + 30)
        self.assertAlmostEqual(left[2] - sim.th, math.radians(31 + sim.mount_errors["L"]))
        self.assertAlmostEqual(right[2] - sim.th, -math.radians(31 + sim.mount_errors["R"]))

    def test_identified_front_deadzone_and_distance_dropout(self):
        sim = S.Sim(S.Case("wood", S.M_L, (2, 0), (0, 2), sensor=S.Sensor(noise=0, mounting_error=0)))
        sim.raycast = lambda *args: 0
        sim.sample()
        self.assertEqual(sim.raw_readings["F"], 50)
        rates = []
        for distance in (80, 220):
            sim.raycast = lambda *args: distance
            invalid = 0
            for _ in range(3000):
                sim.sample()
                invalid += sim.raw_readings["R"] < 0
            rates.append(invalid / 3000)
        self.assertLess(rates[0], 0.04)
        self.assertGreater(rates[1], 0.26)
        self.assertLess(rates[1], 0.34)
        sim.raycast = lambda *args: 301
        sim.sample()
        self.assertEqual(sim.raw_readings["R"], -1)

    def test_cell_callback_runs_while_moving(self):
        sim = S.Sim(S.Case("callback", S.M_L, (2, 0), (0, 2)))
        observations = []
        list(
            sim.run_cells(
                1, 150, on_cell=lambda index, sides, front: observations.append((index, sim.travelled, sim.t))
            )
        )
        self.assertEqual(len(observations), 1)
        self.assertLess(observations[0][1], sim.travelled)
        self.assertEqual(sim.explore_v, 0)
        self.assertLess(observations[0][2], sim.t)

    def test_side_sample_is_taken_before_target_center(self):
        sim = S.Sim(S.Case("sample", S.M_L, (2, 0), (0, 2)))
        gen = sim.run_cells(1, 100)
        start = sim.travelled
        for _ in range(2000):
            next(gen)
            if sim.approach_sides:
                break
        self.assertIn(1, sim.approach_sides)
        self.assertAlmostEqual(sim.travelled - start, sim.pitch - sim.c.fw.side_look_ahead + 12, delta=1)
        gen.close()

    def test_three_zero_cell_moves_fail_without_reversing(self):
        class Blocked(S.Sim):
            attempts = 0

            def tick(self):
                self.t += self.DT
                self.nsamples += 1
                self.readings = dict(L=-1, F=-1, R=-1)

            def rotate_precise_to(self, target):
                self.th = target
                yield from ()

            def run_cells(self, *args, **kwargs):
                self.attempts += 1
                self.assert_forward = not kwargs.get("reverse", False)
                yield from ()
                return 0, True

        sim = Blocked(S.Case("blocked", S.M_T, (2, 1), (0, 0)))
        with self.assertRaisesRegex(S.MotionFailure, "sin avance"):
            list(sim.explore((2, 1), 0))
        self.assertEqual(sim.attempts, 3)
        self.assertTrue(sim.assert_forward)
        # Cada pared del intento anterior sigue en su direccion absoluta;
        # no se reinterpretan izquierda/derecha tras cambiar de rumbo.
        known = set()
        for _, pos, heading, left, front, right, *_ in sim.sensed:
            if known:
                self.assertEqual(left, (heading + 3) % 4 in known)
                self.assertEqual(right, (heading + 1) % 4 in known)
                self.assertTrue(front)
            if front:
                known.add(heading)


if __name__ == "__main__":
    unittest.main()
