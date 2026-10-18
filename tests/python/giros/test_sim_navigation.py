"""Native parity and observable-only safety regression tests."""

import re
import unittest

from raton_sim.giros import native as N
from raton_sim.simulacion import simulator as S


class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Missing native core is an error, not a silently skipped comparison.
        cls.native = N.NativeNavigation()

    def test_imports_real_firmware_configuration(self):
        text = N.configuration_text()
        text = re.sub(r"(KP_WALL\s*=)[^;]+;", r"\g<1> 0.27f;", text)
        fw = N.firmware_config(S, config_text=text)
        self.assertEqual(fw.kp_wall, 0.27)
        self.assertEqual(fw.cfg_side_ahead, 30)
        self.assertEqual(fw.turn_timeout, 3)

    def test_sweep_unknown_is_not_replaced_by_true_map(self):
        robot = S.Robot()
        fw = N.firmware_config(S)
        missing = [(-1, False, 0, 1000)] * 3
        self.assertEqual(self.native.turn(missing, robot, fw, 90), 0)
        # A visible front wall crossing the body must reject all entry candidates.
        observations = [(-1, False, 0, 1000), (0, True, 1, 0), (-1, False, 0, 1000)]
        self.assertIsNone(self.native.turn(observations, robot, fw, 90))


if __name__ == "__main__":
    unittest.main()
