"""Native parity and observable-only safety regression tests."""
import ctypes
import json
import math
import os
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import navigation as N
import sim_extremos as S
from navigation_experiments import Predictor


class NavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Missing native core is an error, not a silently skipped comparison.
        cls.native=N.NativeNavigation()

    def test_imports_real_firmware_configuration(self):
        fw=N.firmware_config(S)
        self.assertEqual(fw.kp_wall,.15)
        self.assertEqual(fw.cfg_side_ahead,30)
        self.assertEqual(fw.turn_timeout,3)

    def test_features_match_firmware(self):
        rng=random.Random(8)
        for _ in range(20):
            h=[[(rng.uniform(0,400),rng.choice((True,False)),rng.random(),rng.uniform(0,2000))
                for i in range(3)] for t in range(3)]
            values=[v for frame in h for sensor in frame for v in sensor]
            out=(ctypes.c_float*43)()
            self.native.lib.raton_features(self.native.array(values),120,-40,3,50,2,out)
            reference=N.features(h,120,-40,3,50,2)
            for a,b in zip(out,reference):self.assertAlmostEqual(a,b,places=6)

    def test_sweep_unknown_is_not_replaced_by_true_map(self):
        robot=S.Robot();fw=N.firmware_config(S)
        missing=[(-1,False,0,1000)]*3
        self.assertEqual(self.native.turn(missing,robot,fw,90),0)
        # A visible front wall crossing the body must reject all entry candidates.
        observations=[(-1,False,0,1000),(0,True,1,0),(-1,False,0,1000)]
        self.assertIsNone(self.native.turn(observations,robot,fw,90))

    def test_missing_inference_falls_back_to_control(self):
        case=S.Case('test',S.M_L,(2,0),(0,2),controller='neural')
        sim=S.Sim(case);sim.predictor=lambda x:float('nan');sim.risk_threshold=.1
        self.assertTrue(sim.action_allowed(90))

    def test_shadow_model_cannot_reject_an_action(self):
        sim=S.Sim(S.Case('test',S.M_L,(2,0),(0,2),controller='corrected'))
        sim.predictor=lambda x:1;sim.risk_threshold=.1
        self.assertTrue(sim.action_allowed(90))

    def test_ranking_predictor_cannot_veto_all_routes(self):
        from dataclasses import replace
        fw=replace(N.firmware_config(S),risk_rank=True)
        sim=S.Sim(S.Case('rank',S.M_L,(2,0),(0,2),controller='neural',fw=fw))
        sim.predictor=lambda x:1.0;sim.risk_threshold=.1
        self.assertTrue(sim.action_allowed(0))
        self.assertTrue(sim.action_allowed(90))
        self.assertTrue(sim.action_allowed(-90))

    def test_trained_inference_matches_firmware(self):
        path=Path(__file__).resolve().parents[1]/'tools/models/risk_model.json'
        model=json.loads(path.read_text())
        predictor=Predictor(model);rng=random.Random(17)
        for _ in range(100):
            x=[rng.random() for i in range(43)]
            self.assertAlmostEqual(self.native.risk(x),predictor(x),delta=2e-5)
        self.assertTrue(math.isnan(self.native.risk([float('nan')]*43)))
        self.assertFalse(model['enabled'])
        self.assertTrue(set(model['train_mazes']).isdisjoint(model['validation_mazes']))
        self.assertTrue(set(range(200,230)).isdisjoint(model['train_mazes']+model['validation_mazes']))


if __name__=='__main__':unittest.main()
