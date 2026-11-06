"""Wide-apron pre-turn and measured forward alignment before the lower gait."""
import math
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/global_compare'))
from follow_route import NativeStairEntry, PathRunner
from maps import STAGES


def pose(x=-5.57,y=2.,z=.4,yaw=-math.pi/2):
    return dict(x=x,y=y,z=z,yaw=yaw,roll=0.,pitch=0.)


class NativeStairEntryTests(unittest.TestCase):
    def entry(self):return NativeStairEntry(STAGES[7],STAGES[8])

    def approaching(self):
        entry=self.entry();entry.step(pose(-6.2,3.35),-.1,9.9);entry.phase='pre_position';entry.step(pose(-6.2,4.15),0.,10.)
        self.assertEqual(entry.phase,'approach')
        return entry

    def test_checkpoints_preserve_original_flight_and_leave_remote_turn_space(self):
        entry=self.entry()
        np.testing.assert_allclose(entry.checkpoint,[-5.57,2.],atol=1e-12)
        np.testing.assert_allclose(entry.turn_checkpoint,[-6.20,4.15],atol=1e-12)
        self.assertAlmostEqual(entry.heading,-math.pi/2)
        self.assertEqual(STAGES[7]['y'],1.65)
        self.assertEqual(STAGES[8]['frame'][1],1.4)

    def test_recorded_failed_approach_first_moves_west_above_riser_not_up_gait(self):
        p=pose(-5.34476746,1.55160440,.40775946,-3.11839926)
        result=self.entry().step(p,0.,10.)
        self.assertFalse(result['ready']);self.assertEqual(result['phase'],'west_clearance')
        np.testing.assert_allclose(result['maneuver_target_xy'],[-6.20,1.70])

    def test_original_goal_cannot_bypass_remote_turn(self):
        result=self.entry().step(pose(),0.,10.)
        self.assertEqual(result['phase'],'west_clearance');self.assertFalse(result['ready'])

    def test_normal_handoff_targets_west_with_only_riser_clearance_not_person_corridor(self):
        result=self.entry().step(pose(-3.99,1.50,yaw=math.pi),0.,10.)
        self.assertEqual(result['phase'],'west_clearance')
        np.testing.assert_allclose(result['maneuver_target_xy'],[-6.20,1.70])
        self.assertEqual(result['linear_x'],.30)

    def test_current_yield_stop_targets_same_y_west_before_any_north_command(self):
        result=self.entry().step(pose(-4.68,2.65,yaw=math.pi),0.,10.)
        self.assertEqual(result['phase'],'west_clearance')
        np.testing.assert_allclose(result['maneuver_target_xy'],[-6.20,2.65])
        self.assertEqual(result['linear_x'],.30);self.assertAlmostEqual(result['angular_z'],0.)

    def test_actual_yield_retry_yaw_does_not_drive_north_towards_person(self):
        result=self.entry().step(pose(-4.64095,2.60136,yaw=2.6945659),0.,10.)
        self.assertEqual(result['phase'],'west_clearance')
        np.testing.assert_allclose(result['maneuver_target_xy'],[-6.20,2.60136])
        self.assertEqual(result['linear_x'],0.)
        self.assertGreater(result['angular_z'],0.)

    def test_west_translation_requires_measured_heading_within_point_tolerance(self):
        outside=self.entry().step(pose(-4.68,2.65,yaw=math.pi-.13),0.,10.)
        inside=self.entry().step(pose(-4.68,2.65,yaw=math.pi-.10),0.,10.)
        self.assertEqual(outside['linear_x'],0.)
        self.assertEqual(inside['linear_x'],.30)
        self.assertLess(abs(inside['linear_x']*math.sin(math.pi-.10)),.036)

    def test_north_pre_turn_target_is_not_activated_before_measured_west_clearance(self):
        entry=self.entry();entry.step(pose(-4.68,2.65,yaw=math.pi),0.,10.)
        blocked=entry.step(pose(-5.75,2.65,yaw=math.pi),1.,11.)
        self.assertEqual(blocked['phase'],'west_clearance')
        cleared=entry.step(pose(-6.12,2.65,yaw=math.pi),2.,12.)
        self.assertEqual(cleared['phase'],'pre_position')
        np.testing.assert_allclose(cleared['maneuver_target_xy'],[-6.2,4.15])

    def test_near_riser_below_verified_apron_start_fails_without_moving(self):
        result=self.entry().step(pose(-4.7666,1.2339,yaw=-2.826),0.,10.)
        self.assertIn('outside the verified flat apron',result['failure_reason'])
        self.assertEqual(result['linear_x'],0.);self.assertEqual(result['angular_z'],0.)

    def test_stage7_activates_remote_manoeuvre_before_old_path_chase(self):
        for p in [pose(-3.99,1.50,yaw=math.pi),pose(-4.8067,1.5873,yaw=-1.50)]:
            calls=[]
            runner=SimpleNamespace(yielding=False,escaping=False,state='running',start_stage=0,start=100.,received=200.,pose=p,
                stages=STAGES,i=7,native_entry=None,
                atomic=lambda *args:None,
                event=lambda name,**data:calls.append((name,data)),
                align_native_entry=lambda now:calls.append(('entry_tick',now)))
            # No legs/raw path are provided: reaching the old path-chase
            # branch would fail this regression from the actual retry pose.
            with patch('follow_route.time.time',return_value=100.),patch('follow_route.time.monotonic',return_value=200.):
                PathRunner.tick(runner)
            self.assertIsInstance(runner.native_entry,NativeStairEntry)
            self.assertEqual(calls[0][0],'native_entry_centering_started')
            self.assertEqual(calls[1][0],'entry_tick')

    def test_large_pivot_runs_only_on_remote_apron(self):
        entry=self.entry();entry.step(pose(-6.2,3.35,yaw=-math.pi),-.1,9.9);entry.phase='pre_position';result=entry.step(pose(-6.2,4.15,yaw=-math.pi),0.,10.)
        self.assertEqual(result['phase'],'pre_heading')
        self.assertEqual(result['linear_x'],0.);self.assertGreater(result['angular_z'],.5)

    def test_north_approach_turns_before_orbiting_small_checkpoint(self):
        entry=self.entry();entry.step(pose(-6.12,3.35,yaw=math.pi/2),-.1,9.9);entry.phase='pre_position';entry.step(pose(-6.12,3.8,yaw=math.pi/2),0.,10.)
        result=entry.step(pose(-6.15,4.02,yaw=math.pi/2),1.,11.)
        self.assertEqual(result['phase'],'pre_heading')
        self.assertEqual(result['linear_x'],0.)
        self.assertGreaterEqual(abs(result['angular_z']),.85)

    def test_old_near_pedestrian_turn_pose_is_not_accepted_as_wide_turn_pose(self):
        entry=self.entry();entry.step(pose(-6.12,3.35,yaw=math.pi/2),-.1,9.9);entry.phase='pre_position';entry.step(pose(-6.12,3.8,yaw=math.pi/2),0.,10.)
        result=entry.step(pose(-5.74,3.575,yaw=-.702),1.,11.)
        self.assertEqual(result['phase'],'pre_position')
        self.assertFalse(result['ready'])

    def test_apron_pivot_drift_does_not_chase_small_xy_goal(self):
        entry=self.entry();entry.step(pose(-6.2,3.35,yaw=-math.pi),-.1,9.9);entry.phase='pre_position';entry.step(pose(-6.2,4.15,yaw=-math.pi),0.,10.)
        result=entry.step(pose(-6.08,3.925,yaw=-2.6),1.,11.)
        self.assertEqual(result['phase'],'pre_heading');self.assertFalse(result['ready'])
        self.assertEqual(result['linear_x'],0.);self.assertGreater(result['angular_z'],0.)

    def test_forward_carrot_corrects_x_without_turning_back_at_final_goal(self):
        result=self.approaching().step(pose(-6.02,2.9),1.,11.)
        self.assertEqual(result['phase'],'approach');self.assertEqual(result['linear_x'],.30)
        self.assertGreater(result['angular_z'],0.)
        self.assertGreater(result['maneuver_target_xy'][0],-6.02)
        self.assertLess(result['maneuver_target_xy'][1],2.9)

    def test_pre_turn_outside_measured_apron_returns_to_remote_target(self):
        entry=self.entry();entry.step(pose(-6.2,3.35,yaw=-math.pi),-.1,9.9);entry.phase='pre_position';entry.step(pose(-6.2,4.15,yaw=-math.pi),0.,10.)
        result=entry.step(pose(-6.2,2.4,yaw=-2.6),1.,11.)
        self.assertEqual(result['phase'],'pre_position');self.assertFalse(result['ready'])
        np.testing.assert_allclose(result['maneuver_target_xy'],[-6.2,4.15])

    def test_good_heading_cannot_relax_final_lateral_gate(self):
        result=self.approaching().step(pose(x=-5.72),1.,11.)
        self.assertEqual(result['phase'],'approach');self.assertFalse(result['ready'])
        self.assertEqual(result['linear_x'],.30)
        self.assertLess(result['maneuver_target_xy'][1],2.)

    def test_small_terminal_heading_error_corrects_with_zero_forward_velocity(self):
        result=self.approaching().step(pose(yaw=-math.pi/2-.2),1.,11.)
        self.assertEqual(result['phase'],'terminal_heading');self.assertFalse(result['ready'])
        self.assertEqual(result['linear_x'],0.);self.assertGreater(result['angular_z'],0.)

    def test_recorded_stand_shift_cannot_trigger_repeated_early_settling(self):
        entry=self.approaching()
        before=entry.step(pose(y=2.0858),1.,11.)
        self.assertEqual(before['phase'],'approach')
        self.assertEqual(before['linear_x'],.30)
        arrived=entry.step(pose(y=2.003),1.5,11.5)
        self.assertEqual(arrived['phase'],'settling')
        shifted=entry.step(pose(y=2.035),2.,12.)
        self.assertEqual(shifted['phase'],'settling')
        self.assertFalse(shifted['ready'])

    def test_large_terminal_turn_fails_without_pivoting_near_riser(self):
        entry=self.approaching();entry.step(pose(yaw=-math.pi/2-.2),1.,11.)
        result=entry.step(pose(yaw=-math.pi),1.1,11.1)
        self.assertFalse(result['ready']);self.assertIn('wide-floor pre-turn',result['failure_reason'])
        self.assertEqual(result['linear_x'],0.);self.assertEqual(result['angular_z'],0.)

    def test_lost_forward_alignment_or_exceeded_stop_region_never_drives_toward_riser(self):
        for p in [pose(y=1.79),pose(x=-6.7,y=2.7),pose(y=2.7,yaw=-2.5)]:
            result=self.approaching().step(p,1.,11.)
            self.assertFalse(result['ready']);self.assertIn('failure_reason',result,p)
            self.assertEqual(result['linear_x'],0.,p)

    def test_wrong_height_waits_without_commands_or_claiming_ready(self):
        result=self.approaching().step(pose(z=.81),1.,11.)
        self.assertFalse(result['ready']);self.assertEqual(result['linear_x'],0.)
        self.assertEqual(result['angular_z'],0.)
        self.assertEqual(result['reason'],'waiting_for_level_floor_pose')

    def test_repeated_timer_cache_does_not_count_as_stationary_observations(self):
        entry=self.approaching()
        for i in range(30):result=entry.step(pose(),1.+i*.05,11.)
        self.assertFalse(result['ready']);self.assertEqual(result['stationary_observations'],1)

    def test_distinct_stationary_observations_confirm_final_alignment(self):
        entry=self.approaching()
        for i in range(17):result=entry.step(pose(),1.+i*.05,11.+i*.05)
        self.assertTrue(result['ready']);self.assertGreaterEqual(result['stationary_observations'],5)
        self.assertEqual(result['linear_x'],0.);self.assertEqual(result['angular_z'],0.)

    def test_settling_drift_requires_new_measured_confirmation(self):
        entry=self.approaching();entry.step(pose(),1.,11.);entry.step(pose(),1.2,11.2)
        displaced=entry.step(pose(x=-5.72),1.3,11.3)
        self.assertEqual(displaced['phase'],'approach')
        restored=entry.step(pose(),2.,12.)
        self.assertFalse(restored['ready']);self.assertEqual(restored['stationary_observations'],1)
        self.assertEqual(entry.settled_since,2.)

    def test_guard_wait_preserves_timeout_allowance_without_claiming_ready(self):
        entry=self.entry();p=pose(-6.2,4.15,yaw=-math.pi)
        entry.step(pose(-6.2,3.35,yaw=-math.pi),-.1,9.9);entry.phase='pre_position';entry.step(p,0.,10.)
        for i in range(1,101):result=entry.step(p,float(i),10.+i,yielding=True)
        self.assertNotIn('failure_reason',result);self.assertFalse(result['ready'])
        self.assertGreater(entry.started,90.)

    def test_stalled_pre_turn_measurements_fail_instead_of_completing(self):
        entry=self.entry();p=pose(-6.2,4.15,yaw=-math.pi)
        entry.step(pose(-6.2,3.35,yaw=-math.pi),-.1,9.9);entry.phase='pre_position';entry.step(p,0.,10.);result=entry.step(p,31.,41.)
        self.assertFalse(result['ready']);self.assertIn('no measured progress',result['failure_reason'])

    def test_unicycle_plus_turn_translation_converges_from_real_failed_poses(self):
        # Independent kinematics and yaw-only translation are helper checks;
        # they are not a claim of real RL/Office navigation success.
        initial_poses=[pose(-5.345,1.55,yaw=-3.118),pose(-5.0,2.4,yaw=-.55),
                       pose(-5.67,1.925,yaw=-2.599),pose(-4.785,1.574,.361,-1.84)]
        for initial in initial_poses:
            entry=self.entry();p=initial.copy();dt=.05;phases=set()
            for i in range(1800):
                result=entry.step(p,i*dt,10.+i*dt);phases.add(result['phase'])
                self.assertNotIn('failure_reason',result,(initial,p,result))
                self.assertLessEqual(abs(result['angular_z']),.90)
                self.assertLessEqual(result['linear_x'],.30)
                if result['ready']:break
                p['yaw']+=result['angular_z']*dt
                drift=.15*abs(result['angular_z']) if result['linear_x']==0. else 0.
                p['x']+=(result['linear_x']+drift)*math.cos(p['yaw'])*dt
                p['y']+=(result['linear_x']+drift)*math.sin(p['yaw'])*dt
            self.assertTrue(result['ready'],(initial,p,result))
            self.assertIn('pre_heading',phases);self.assertIn('approach',phases)
            self.assertLessEqual(abs(p['x']+5.57),.12)
            self.assertLessEqual(abs(entry.angle(p['yaw']+math.pi/2)),.12)

    def test_large_first_stair_cannot_use_lower_flight_manoeuvre(self):
        with self.assertRaises(ValueError):NativeStairEntry(STAGES[0],STAGES[1])


if __name__=='__main__':unittest.main()
