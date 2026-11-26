#!/usr/bin/env python3
"""Expand the real launch with ROS, without starting inference or controllers."""
import json
import os
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from roslaunch.config import load_config_default
import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT/'launch/navigation.launch'


class NavigationLaunchTest(unittest.TestCase):
    def load(self, *args):
        config = load_config_default([(str(LAUNCH), list(args))], 11311, verbose=False)
        return config, {name: param.value for name, param in config.params.items()}

    def test_modes_and_reference_controls_reach_actual_teb(self):
        snapshots = {}
        for fusion, profile in (('layers', 'current'), ('layers', 'matched'), ('points', 'matched')):
            for reference in ('state_machine', 'latest', 'avoidance_commit'):
                with self.subTest(fusion=fusion, profile=profile, reference=reference):
                    config, params = self.load('fusion_mode:='+fusion, 'comparison_profile:='+profile,
                                               'reference_mode:='+reference, 'path_timeout:=1.25',
                                               'shrink_horizon_backup:=false')
                    prefix = '/placenav_teb_bridge/'
                    self.assertEqual(params[prefix+'reference_mode'], reference)
                    self.assertEqual(params[prefix+'path_timeout'], 1.25)
                    self.assertIs(params[prefix+'TebLocalPlannerROS/shrink_horizon_backup'], False)
                    self.assertEqual(params[prefix+'cmd_vel_topic'], '/fusion_validation/cmd_vel')
                    layers = [p['name'] for p in params[prefix+'local_costmap/plugins']]
                    self.assertEqual(layers, ['fused', 'inflation'] if fusion == 'points'
                                     else ['d435', 'unik3d', 'inflation'])
                    self.assertEqual(sum(n.name == 'placenav_teb_bridge' for n in config.nodes), 1)
                    for gate in ('d435_gate', 'unik3d_gate'):
                        self.assertEqual(sum(n.name == gate for n in config.nodes), 1)
                    snapshots[fusion+'_'+profile+'_'+reference] = params
        if os.environ.get('LAUNCH_OPTIONS_OUTPUT'):
            Path(os.environ['LAUNCH_OPTIONS_OUTPUT']).write_text(json.dumps(snapshots, indent=2)+'\n')

    def test_default_navigation_controls(self):
        _, params = self.load()
        bridge = ET.parse(ROOT.parent/'placenav_teb_bridge/launch/bridge.launch').getroot()
        defaults = {arg.attrib['name']: arg.attrib.get('default') for arg in bridge.findall('arg')}
        # Navigation intentionally uses latest; standalone bridge uses state_machine.
        self.assertEqual(params['/placenav_teb_bridge/reference_mode'], 'latest')
        self.assertEqual(params['/placenav_runner/motion_model'], 'odometry')
        self.assertEqual(params['/placenav_runner/subgoal_update'], 'arrival')
        self.assertIs(params['/placenav_runner/local_pose'], True)
        self.assertEqual(params['/placenav_runner/topomap_dir'], 'whill_odometry_tsukuba_3.5')
        config, _ = self.load()
        self.assertEqual(sum(n.name == 'local_goal_visualizer' for n in config.nodes), 1)
        for launch in [LAUNCH, ROOT/'launch/navigation_debug.launch']:
            config = load_config_default([(str(launch), [])], 11311, verbose=False)
            self.assertIs(config.params['/placenav_runner/local_pose'].value, True)
            self.assertEqual(config.params['/placenav_runner/subgoal_update'].value, 'arrival')
            self.assertEqual(sum(n.name == 'local_goal_visualizer' for n in config.nodes), 1)
            config = load_config_default([(str(launch), ['local_pose:=false',
                'subgoal_update:=nearest'])], 11311, verbose=False)
            self.assertIs(config.params['/placenav_runner/local_pose'].value, False)
            self.assertEqual(config.params['/placenav_runner/subgoal_update'].value, 'nearest')
            self.assertFalse(any(n.name == 'local_goal_visualizer' for n in config.nodes))
        self.assertEqual(defaults['reference_mode'], 'state_machine')
        self.assertEqual(params['/placenav_teb_bridge/path_timeout'], float(defaults['path_timeout']))
        self.assertEqual(params['/placenav_teb_bridge/TebLocalPlannerROS/shrink_horizon_backup'],
                         defaults['shrink_horizon_backup'] == 'true')
        self.assertEqual(params['/placenav_teb_bridge/comparison_profile'], 'current')

    def test_real_and_debug_select_separate_tuning_files(self):
        for launch, filename in [(LAUNCH, 'placenav.yaml'),
                                 (ROOT/'launch/navigation_debug.launch', 'placenav_dev.yaml')]:
            with self.subTest(launch=launch.name):
                values = yaml.safe_load((ROOT/'config'/filename).read_text())
                config = load_config_default([(str(launch), [])], 11311, verbose=False)
                prefix = '/placenav_runner/'
                self.assertEqual(Path(config.params[prefix+'placenav_config'].value).name, filename)
                for name, value in values.items():
                    self.assertEqual(config.params[prefix+name].value, value, name)
                self.assertNotIn(prefix+'goal_eval_yaw_tolerance', config.params)
                # Remove an earlier radian override when reusing the ROS master.
                self.assertIn(prefix, config.clear_params)

    def test_yaml_tuning_and_explicit_override_reach_navigation_command(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('yaml_session_runner', ROOT/'scripts/session_runner.py')
        runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
        values = yaml.safe_load((ROOT/'config/placenav.yaml').read_text())
        values.update(odom_noise=.25, pass_threshold=.7, lookahead=2, local_pose_reset=False,
                      goal_eval_yaw_tolerance_deg=35.)
        with tempfile.TemporaryDirectory(prefix='placenav-config-') as directory:
            path = Path(directory)/'custom config.yaml'
            path.write_text(yaml.safe_dump(values))
            for launch in [LAUNCH, ROOT/'launch/navigation_debug.launch']:
                for override in [False, True]:
                    with self.subTest(launch=launch.name, override=override):
                        options = ['placenav_config:='+str(path), 'subgoal_update:=nearest']
                        if override:
                            options += ['odom_noise:=0.15', 'pass_threshold:=0.6', 'local_pose_reset:=true',
                                        'goal_eval_yaw_tolerance:=0.2']
                        config = load_config_default([(str(launch), options)], 11311, verbose=False)
                        prefix = '/placenav_runner/'
                        params = {name[len(prefix):]: p.value for name, p in config.params.items()
                                  if name.startswith(prefix)}
                        params['placenav_src'] = str(ROOT.parent)
                        command, _ = runner.navigation_command(params)
                        self.assertIs(params['local_pose_reset'], override)
                        self.assertIn('--local-pose-reset' if override else '--no-local-pose-reset', command)
                        import math
                        expected = {'--odom-noise': .15 if override else .25,
                                    '--pass-threshold': .6 if override else .7,
                                    '--lookahead': 2,
                                    '--goal-eval-yaw-tolerance': .2 if override else math.radians(35.)}
                        for flag, value in expected.items():
                            self.assertEqual(float(command[command.index(flag)+1]), value, flag)

    def test_mode_reset_patterns_reach_the_actual_parser_in_both_launches(self):
        import importlib.util
        from unittest.mock import patch
        import sys
        def module(name, path):
            spec = importlib.util.spec_from_file_location(name, path)
            value = importlib.util.module_from_spec(spec); spec.loader.exec_module(value)
            return value
        runner = module('reset_runner', ROOT/'scripts/session_runner.py')
        parser = module('reset_parser', ROOT.parent/'placenav/parser.py')
        required = ['--robot', 'whill', '--robot-config-path', 'robots.yaml',
                    '--model-config-path', 'models.yaml', '--topomap-base-dir', 'maps',
                    '--model-weight-dir', 'weights']
        for launch in [LAUNCH, ROOT/'launch/navigation_debug.launch']:
            for mode in ['nearest', 'passage', 'arrival', 'arrival_turn', 'arrival_turn_bend']:
                for enabled in [False, True]:
                    with self.subTest(launch=launch.name, mode=mode, enabled=enabled):
                        options = ['subgoal_update:='+mode, 'local_pose_reset:='+str(enabled).lower()]
                        if mode == 'arrival_turn_bend':
                            options.extend(['turn_check_start_deg:=35', 'turn_required_ratio:=0.4'])
                        config = load_config_default([(str(launch), options)], 11311, verbose=False)
                        prefix = '/placenav_runner/'
                        params = {k[len(prefix):]:p.value for k,p in config.params.items() if k.startswith(prefix)}
                        params['placenav_src'] = str(ROOT.parent)
                        command, _ = runner.navigation_command(params)
                        with patch.object(sys, 'argv', ['navigate.py']+required+command[2:]):
                            args = parser.parse_args()
                        self.assertEqual(args.subgoal_update, mode)
                        if mode == 'arrival_turn_bend':
                            self.assertEqual(args.turn_check_start_deg, 35.)
                            self.assertEqual(args.turn_required_ratio, .4)
                        self.assertIs(args.local_pose, True)
                        self.assertIs(args.local_pose_reset, enabled)
                        self.assertEqual(sum(n.name == 'local_goal_visualizer' for n in config.nodes), 1)

    def test_heading_align_reaches_the_parser_in_both_launches(self):
        import importlib.util
        from unittest.mock import patch
        import sys
        def module(name, path):
            spec = importlib.util.spec_from_file_location(name, path)
            value = importlib.util.module_from_spec(spec); spec.loader.exec_module(value)
            return value
        runner = module('align_runner', ROOT/'scripts/session_runner.py')
        parser = module('align_parser', ROOT.parent/'placenav/parser.py')
        required = ['--robot', 'whill', '--robot-config-path', 'robots.yaml',
                    '--model-config-path', 'models.yaml', '--topomap-base-dir', 'maps',
                    '--model-weight-dir', 'weights']
        prefix = '/placenav_runner/'
        for launch in [LAUNCH, ROOT/'launch/navigation_debug.launch']:
            with self.subTest(launch=launch.name, enabled=False):
                config = load_config_default([(str(launch), [])], 11311, verbose=False)
                params = {k[len(prefix):]:p.value for k,p in config.params.items() if k.startswith(prefix)}
                params['placenav_src'] = str(ROOT.parent)
                self.assertNotIn('--heading-align', runner.navigation_command(params)[0])
            with self.subTest(launch=launch.name, enabled=True):
                config = load_config_default([(str(launch), [
                    'heading_align:=true', 'heading_align_start_deg:=30', 'heading_align_done_deg:=12',
                    'heading_align_resize:=800', 'heading_align_max_keypoints:=512'])], 11311, verbose=False)
                params = {k[len(prefix):]:p.value for k,p in config.params.items() if k.startswith(prefix)}
                params['placenav_src'] = str(ROOT.parent)
                command, _ = runner.navigation_command(params)
                with patch.object(sys, 'argv', ['navigate.py']+required+command[2:]):
                    args = parser.parse_args()
                self.assertIs(args.heading_align, True)
                self.assertEqual((args.heading_align_start_deg, args.heading_align_done_deg,
                                  args.heading_align_resize, args.heading_align_max_keypoints), (30., 12., 800, 512))
                with self.assertRaises(ValueError):
                    runner.navigation_command(dict(params, subgoal_update='passage'))
                # The Xavier waypoint role gets the same flags and ignores them.
                xavier, _ = runner.xavier_navigation_command(
                    dict(params, remote_navigation=True, remote_role='waypoint'), str(ROOT.parent))
                with patch.object(sys, 'argv', ['navigate.py']+required+xavier[2:]):
                    self.assertEqual(parser.parse_args().nav_role, 'waypoint')

    def test_frames_reach_gates_tf_batch_maps_teb_and_path_runner(self):
        config, params = self.load('global_frame:=test_odom', 'robot_base_frame:=test_base',
                                   'odom_topic:=/test/odom', 'fusion_mode:=points', 'debug_costmaps:=true')
        for gate in ('d435_gate', 'unik3d_gate'):
            self.assertEqual(params['/obstacles/'+gate+'/fixed_frame'], 'test_odom')
            self.assertEqual(params['/obstacles/'+gate+'/base_frame'], 'test_base')
        for ns in ('/placenav_teb_bridge', '/obstacles/source_costmaps'):
            self.assertEqual(params[ns+'/local_costmap/global_frame'], 'test_odom')
            self.assertEqual(params[ns+'/local_costmap/robot_base_frame'], 'test_base')
            for layer in ('d435', 'unik3d', 'fused'):
                self.assertEqual(params[ns+'/local_costmap/'+layer+'/cloud/sensor_frame'], 'test_odom')
        self.assertEqual(params['/placenav_teb_bridge/TebLocalPlannerROS/map_frame'], 'test_odom')
        self.assertEqual(params['/placenav_teb_bridge/TebLocalPlannerROS/odom_topic'], '/test/odom')
        self.assertEqual(params['/placenav_runner/global_frame'], 'test_odom')
        self.assertEqual(params['/placenav_runner/robot_base_frame'], 'test_base')
        self.assertFalse(any(n.name == 'c2_floor_candidate_tf' for n in config.nodes))
        self.assertEqual(params['/obstacles/unik3d_gate/frame_override'], '')
        self.assertEqual(sum(n.name == 'comparison' for n in config.nodes), 1)

    def test_navigation_tuning_and_recording_parameters(self):
        _, params = self.load('start_node_idx:=5', 'goal_node_idx:=31', 'lookahead:=2', 'nomad_seed:=7',
                              'subgoal_update:=nearest',
                              'img_size:=[96,96]', 'recompute_place_recognition_db:=true', 'diagnostics:=true')
        for key, value in dict(start_node_idx=5, goal_node_idx=31, lookahead=2, nomad_seed=7,
                               img_size=[96, 96], recompute_place_recognition_db=True).items():
            self.assertEqual(params['/placenav_runner/'+key], value)
        self.assertTrue(params['/placenav_runner/diagnostics'])

    def test_recording_is_a_plain_rosbag_node_with_only_the_listed_topics(self):
        config, params = self.load('diagnostics:=true', 'bag_dir:=/tmp/bags', 'bag_prefix:=trial')
        recorder, = [n for n in config.nodes if n.name == 'session_recorder']
        self.assertEqual((recorder.package, recorder.type), ('rosbag', 'record'))
        self.assertFalse(any(name.startswith('/session_recorder/') for name in params))
        args = recorder.args.split()
        self.assertEqual(args[:5], ['-o', '/tmp/bags/trial', '--lz4', '--buffsize', '512'])
        topics = args[5:]
        self.assertIn('/toponav/goal_evaluation', topics)
        self.assertIn('/toponav/local_goal_pose', topics)
        self.assertIn('/toponav/heading_align', topics)
        self.assertFalse(any(t.startswith('/rwrc26/') for t in topics))  # diagnostics add no topics
        self.assertTrue(all(t.startswith('/') for t in topics))
        self.assertEqual(len(topics), len(set(topics)))
        config, _ = self.load('record:=false')
        self.assertFalse(any(n.name == 'session_recorder' for n in config.nodes))

    def test_invalid_reference_controls_fail_before_nodes_start(self):
        for argument in ('reference_mode:=typo', 'path_timeout:=-1', 'path_timeout:=nan', 'path_timeout:=inf'):
            with self.subTest(argument=argument), self.assertRaises(Exception):
                self.load(argument)

    def test_remote_navigation_requires_path_timeout(self):
        for argument in ('path_timeout:=0.0', 'path_timeout:=0'):
            with self.subTest(argument=argument), self.assertRaises(Exception):
                self.load('remote_navigation:=true', argument)
        _, params = self.load('remote_navigation:=true', 'path_timeout:=1.0')
        self.assertIs(params['/placenav_runner/remote_navigation'], True)
        self.assertEqual(params['/placenav_teb_bridge/path_timeout'], 1.0)
        _, params = self.load()
        self.assertIs(params['/placenav_runner/remote_navigation'], False)

    def test_explicit_drive_switch(self):
        _, params = self.load('drive_robot:=true', 'robot_cmd_vel_topic:=/test/drive')
        self.assertEqual(params['/placenav_teb_bridge/cmd_vel_topic'], '/test/drive')

    def test_odometry_options_reach_runner_and_debug_bag_input(self):
        _, params = self.load('odom_topic:=/vo/odom', 'subgoal_update:=passage',
                              'pass_threshold:=0.6', 'pass_confirmations:=3',
                              'pass_local_mass:=0.9', 'final_goal_tolerance:=0.4',
                              'odom_max_gap:=0.15', 'odom_noise:=0.2')
        for key, value in dict(motion_model='odometry', odom_topic='/vo/odom',
                               odom_max_gap=.15, odom_noise=.2, subgoal_update='passage',
                               pass_threshold=.6, pass_confirmations=3,
                               pass_local_mass=.9, final_goal_tolerance=.4).items():
            self.assertEqual(params['/placenav_runner/'+key], value)
        debug = ROOT/'launch/navigation_debug.launch'
        config = load_config_default([(str(debug), ['bag:=/tmp/example.bag',
            'subgoal_update:=passage', 'odom_topic:=/vo/odom', 'rviz:=false'])], 11311, verbose=False)
        self.assertIs(config.params['/use_sim_time'].value, True)
        self.assertEqual(config.params['/placenav_runner/motion_model'].value, 'odometry')
        self.assertEqual(config.params['/placenav_runner/subgoal_update'].value, 'passage')
        import shlex
        player = next(n for n in config.nodes if n.name == 'navigation_bag_playback')
        topics = shlex.split(player.args)[shlex.split(player.args).index('--topics')+1:]
        self.assertIn('/vo/odom', topics)
        self.assertNotIn('/odom', topics)
        _, legacy = self.load('motion_model:=legacy', 'subgoal_update:=nearest', 'local_pose:=false')
        self.assertEqual(legacy['/placenav_runner/motion_model'], 'legacy')

    def test_goal_evaluation_options_reach_both_launch_entry_points(self):
        options = ['goal_eval_position_tolerance:=0.4', 'goal_eval_yaw_tolerance:=0.2',
                   'goal_eval_neighborhood_radius:=3.0', 'subgoal_update:=arrival',
                   'goal_recovery_distance:=5.0', 'goal_recovery_threshold:=0.9', 'local_pose:=true']
        for launch in [LAUNCH, ROOT/'launch/navigation_debug.launch']:
            config = load_config_default([(str(launch), options)], 11311, verbose=False)
            for name, value in [('position_tolerance', .4), ('yaw_tolerance', .2),
                                ('neighborhood_radius', 3.)]:
                self.assertEqual(config.params['/placenav_runner/goal_eval_'+name].value, value)
            for name, value in [('subgoal_update', 'arrival'), ('goal_recovery_distance', 5.),
                                ('goal_recovery_threshold', .9), ('local_pose', True)]:
                self.assertEqual(config.params['/placenav_runner/'+name].value, value)
            self.assertEqual(sum(n.name == 'local_goal_visualizer' for n in config.nodes), 1)
            self.assertEqual(config.params['/local_goal_visualizer/target_frame'].value, 'odom')

    def test_debug_bag_start_is_seconds_into_the_bag(self):
        import shlex
        launch = ROOT/'launch/navigation_debug.launch'
        for value in ['0', '420', '419.5']:
            config = load_config_default([(str(launch), ['bag:=/tmp/example.bag',
                'bag_start:='+value, 'bag_pause:=true', 'rviz:=false'])], 11311, verbose=False)
            player = next(n for n in config.nodes if n.name == 'navigation_bag_playback')
            args = shlex.split(player.args)
            self.assertIn('--start='+value, args)
            self.assertIn('--clock', args)
            self.assertIn('--pause', args)
            self.assertIs(config.params['/use_sim_time'].value, True)
        for value in ['-1', 'nan', 'inf']:
            with self.subTest(value=value), self.assertRaises(Exception):
                load_config_default([(str(launch), ['bag_start:='+value])], 11311, verbose=False)

    def test_clearing_costmaps_stay_inside_gate_marking_band(self):
        import math
        configs = ROOT.parent/'source_costmaps/config'
        debug = ROOT/'launch/navigation_debug.launch'
        prefix = '/placenav_teb_bridge/local_costmap/'
        # Defaults keep the decay-only configs.
        for launch in (LAUNCH, debug):
            params = {k: p.value for k, p in
                      load_config_default([(str(launch), [])], 11311, verbose=False).params.items()}
            for layer in ('d435', 'unik3d'):
                self.assertEqual(params[prefix+layer+'/observation_sources'], 'cloud')
        for launch, name in ((LAUNCH, 'costmap_clearing.yaml'), (debug, 'costmap_dev_clearing.yaml')):
            config = load_config_default([(str(launch), ['costmap_config:='+str(configs/name)])],
                                         11311, verbose=False)
            params = {k: p.value for k, p in config.params.items()}
            for layer, frame in (('d435', 'realsense_color_optical_frame'),
                                 ('unik3d', 'c2_camera_optical_frame')):
                with self.subTest(config=name, layer=layer):
                    base = prefix+layer+'/'
                    clear = base+'cloud_clear/'
                    gate = '/obstacles/'+layer+'_gate/'
                    self.assertEqual(params[base+'observation_sources'], 'cloud cloud_clear')
                    self.assertIs(params[clear+'clearing'], True)
                    self.assertIs(params[clear+'marking'], False)
                    self.assertEqual(params[clear+'model_type'], 0)
                    self.assertEqual(params[clear+'topic'], params[base+'cloud/topic'])
                    self.assertEqual(params[clear+'topic'], params[gate+'output_topic'])
                    self.assertEqual(params[clear+'sensor_frame'], frame)
                    # STVL frustum: corner range == max_z, near plane at
                    # min_z*cos(v/2)*cos(h/2); both must stay in the gate's range band.
                    h, v = params[clear+'horizontal_fov_angle'], params[clear+'vertical_fov_angle']
                    near = params[clear+'min_z']*math.cos(v/2)*math.cos(h/2)
                    self.assertGreaterEqual(near, params[gate+'min_range'])
                    self.assertLessEqual(params[clear+'max_z'], params[gate+'max_range'])
                    self.assertGreater(params[clear+'decay_acceleration'], 0)


if __name__ == '__main__':
    unittest.main()
