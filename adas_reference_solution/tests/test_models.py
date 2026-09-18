"""Analytic benchmarks and physical invariants, independent of plot appearance."""
import unittest
import numpy as np
from numpy.polynomial import Polynomial
from adas import braking, trajectory, tracking, speed


class BrakingTests(unittest.TestCase):
    def test_low_friction_and_distance_identity(self):
        for mu in (.15,.5,.85):
            s,d=braking.scenario(mu)
            self.assertLessEqual(s['brake_mps2'],mu*9.81+1e-12)
            np.testing.assert_allclose(d['travelled_m']+d['remaining_stop_m'],s['stop_distance_m'])
            self.assertAlmostEqual(d['speed_mps'][-1],0)
        self.assertAlmostEqual(braking.scenario(.15)[0]['stop_distance_m'],625/(2*.15*9.81))

    def test_friction_saturation(self):
        for mu in (.15,.5):
            s,d=braking.scenario(mu)
            self.assertEqual(s['lateral_capacity_mps2'],0)
            self.assertIsNone(s['combined_braking_m'])
        self.assertLess(braking.scenario(.85)[0]['combined_braking_m'],braking.scenario(.85)[0]['combined_frozen_m'])


class TrajectoryTests(unittest.TestCase):
    def test_six_boundary_conditions(self):
        initial=(4.,8.,.2); final=(45.,3.,-.1)
        for T in (2.,8.,12.):
            q=trajectory.quintic(T,initial,final)
            for i in range(3):
                self.assertAlmostEqual(q.deriv(i)(0)/T**i,initial[i],places=8)
                self.assertAlmostEqual(q.deriv(i)(1)/T**i,final[i],places=8)

    def test_known_minimum_jerk_solution(self):
        # Rest-to-rest unit displacement: 10u^3-15u^4+6u^5, integral(j²)=720/T^5.
        T=3.; q=trajectory.quintic(T,(0,0,0),(1,0,0))
        np.testing.assert_allclose(q.coef,[0,0,0,10,-15,6],atol=1e-12)
        j=q.deriv(3)/T**3; primitive=(j*j).integ()
        self.assertAlmostEqual(T*(primitive(1)-primitive(0)),720/T**5,places=10)

    def test_stationary_point_not_missed(self):
        p=Polynomial([.24,-1,1]) # ends positive; unsafe at center
        self.assertAlmostEqual(trajectory.extrema(p)[0],-.01)

    def test_infeasible_selection_explicit(self):
        cfg=trajectory.Config(lead_s0=1)
        pool=[trajectory.candidate(4,0,cfg)]
        self.assertIsNone(trajectory.select(pool,filtered=True))

    def test_selected_constraints_and_finite_grid_optimality(self):
        pool=trajectory.candidates(time_step=1.,offset_step=2.)
        selected=trajectory.select(pool,filtered=True)
        self.assertIsNotNone(selected)
        for c in pool:
            if c['feasible']:
                self.assertLessEqual(selected['cost'],c['jerk_integral']+c['T_s']+c['delta_m']**2+1e-10)
        d=trajectory.sample(selected,n=10001)
        self.assertGreaterEqual(np.min(d['margin_m']),-1e-8)


class TrackingTests(unittest.TestCase):
    def test_reference_heading_is_path_slope(self):
        x=np.array([40.,60.,80.,130.,150.]); h=1e-4
        slope=(tracking.reference(x+h)[0]-tracking.reference(x-h)[0])/(2*h)
        np.testing.assert_allclose(np.tan(tracking.reference(x)[1]),slope,atol=1e-9)

    def test_segment_projection_not_waypoint_distance(self):
        i,foot,signed,distance,psi=tracking.project(np.array([.5,2.]),np.array([[0.,0.],[1.,0.]]))
        np.testing.assert_allclose(foot,[.5,0])
        self.assertAlmostEqual(distance,2); self.assertAlmostEqual(signed,-2)

    def test_pursuit_circle_intersection(self):
        point=np.array([0.,0.]); path=np.array([[0.,0.],[10.,0.],[20.,0.]])
        np.testing.assert_allclose(tracking.pursuit_target(point,path,0,point,15),[15,0])

    def test_finite_path_end_does_not_reset(self):
        row,d,_=tracking.simulate(controller='pure_pursuit',duration=24,dt=.05)
        self.assertEqual(row['status'],'path_end')
        self.assertGreater(d['x_m'][-1],449)
        self.assertGreater(np.min(np.diff(d['x_m'])),-1)


class SpeedTests(unittest.TestCase):
    def test_exact_straight_and_arc_geometry(self):
        x,y=speed.geometry([0,400,1000])
        np.testing.assert_allclose(x[:2],[0,400]); np.testing.assert_allclose(y[:2],[0,0])
        self.assertAlmostEqual(x[2],400+np.sin(3)/.005)
        self.assertAlmostEqual(y[2],(1-np.cos(3))/.005)

    def test_curve_and_traffic_units(self):
        r,t,c=speed.limits(np.array([500.]),.01)
        self.assertAlmostEqual(r[0],np.sqrt(2))
        self.assertAlmostEqual(t[0],50/3.6)

    def test_full_interval_constraints_and_initial_speed(self):
        row,d=speed.plan()
        self.assertEqual(row['status'],'feasible'); self.assertEqual(d['speed_mps'][0],25)
        self.assertLessEqual(row['max_speed_excess_mps'],1e-8)
        self.assertLessEqual(row['max_lateral_accel_mps2'],1.5+1e-8)
        self.assertGreaterEqual(row['min_accel_mps2'],-2-1e-8)
        self.assertLessEqual(row['max_accel_mps2'],1+1e-8)
        np.testing.assert_allclose(np.diff(d['s_m']),.5*(d['speed_mps'][:-1]+d['speed_mps'][1:])*np.diff(d['t_s']),atol=1e-10)

    def test_infeasibility_is_not_silently_clipped(self):
        for args in [(0,0,45),(.01,.01,25)]:
            row,d=speed.plan(*args)
            self.assertEqual(row['status'],'infeasible_initial_speed'); self.assertIsNone(d)

    def test_zero_lateral_limit_cannot_traverse_curve(self):
        row,d=speed.plan(2,0,0)
        self.assertEqual(row['status'],'blocked_zero_speed_interval'); self.assertIsNone(d)

    def test_transition_limit_met(self):
        row,d=speed.plan()
        idx=np.where(d['s_m']==400)[0][0]
        self.assertAlmostEqual(d['speed_mps'][idx],50/3.6)
        # Eq.27 predicts start of braking at 291.9753 m for this transition.
        trigger=(25**2-(50/3.6)**2)/4
        s=400-trigger
        i=np.searchsorted(d['s_m'],s)
        self.assertAlmostEqual(d['speed_mps'][i-1],25)
        self.assertLess(d['speed_mps'][i],25)


if __name__=='__main__':
    unittest.main()
