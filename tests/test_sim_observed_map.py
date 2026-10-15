"""Observable-map guard regressions; hidden geometry never becomes a control input."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import sim_extremos as S
from navigation_candidates import ObservedMapSim

class ObservedMapTests(unittest.TestCase):
    def make(self):
        return ObservedMapSim(S.Case('map',S.M_L,(2,0),(0,2)),seed=1)

    def test_true_position_cannot_change_observed_guard(self):
        sim=self.make()
        sim.sensed=[(0,(2,0),0,True,False,False,{},('unused truth',))]
        before=sim.observed_segments()
        sim.x=-100000;sim.y=100000
        self.assertEqual(sim.observed_segments(),before)
        self.assertEqual(len(before),4)

    def test_unknown_maze_is_not_a_map_prior(self):
        sim=self.make()
        self.assertFalse(sim.nav_walls)
        self.assertEqual(sim.observed_segments(),[])

    def test_encoder_translation_updates_navigation_pose(self):
        sim=self.make();before=sim.nav_y
        self.assertTrue(sim.try_move(sim.x,sim.y+1,sim.th))
        self.assertAlmostEqual(sim.nav_y,before+1)
        self.assertAlmostEqual(sim.nav_x,(2+.5)*sim.pitch)

    def test_whole_body_and_margin_block_an_observed_wall(self):
        sim=self.make()
        self.assertFalse(sim.clear_observed_pose([((80,-100),(80,100))],0,0,0))
        self.assertTrue(sim.clear_observed_pose([((100,-100),(100,100))],0,0,0))
        self.assertFalse(sim.clear_observed_pose([((-20,0),(100,0))],0,0,math.pi/2))

if __name__=='__main__':unittest.main()
