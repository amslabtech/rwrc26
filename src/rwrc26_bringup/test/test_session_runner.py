#!/usr/bin/env python3
"""No cameras, model inference or robot command publishers are used here."""
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/session_runner.py'
spec = importlib.util.spec_from_file_location('session_runner', SCRIPT)
runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)


class SessionRunnerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='rwrc26-session-')
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_navigation_keeps_argument_boundaries_and_working_directory(self):
        directory = self.root/'PlaceNav source'; directory.mkdir()
        (directory/'navigate_whill.sh').write_text('''#!/bin/bash
python3 - "$@" <<'PY'
import json,os,sys
open('captured.json','w').write(json.dumps({'cwd':os.getcwd(),'args':sys.argv[1:]}))
PY
''')
        route = 'route with spaces; touch unexpected_file'
        command, cwd = runner.navigation_command({'placenav_src': str(directory), 'topomap_dir': route,
                                                 'visualization': False, 'window_radius': 3})
        subprocess.run(command, cwd=cwd, check=True)
        captured = json.loads((directory/'captured.json').read_text())
        self.assertEqual(captured['cwd'], str(directory))
        self.assertEqual(captured['args'][captured['args'].index('--topomap-dir')+1], route)
        self.assertIn('--no-visualization', captured['args'])
        self.assertEqual(captured['args'][captured['args'].index('--wp-model')+1], 'nomad')
        self.assertEqual(captured['args'][captured['args'].index('--pr-model')+1], 'salad')
        self.assertEqual(captured['args'][captured['args'].index('--motion-model')+1], 'odometry')
        self.assertFalse((directory/'unexpected_file').exists())
        self.assertFalse(any(a.startswith('__') for a in captured['args']))

    def test_missing_navigation_script_rejected(self):
        with self.assertRaises(ValueError):
            runner.navigation_command({'placenav_src': str(self.root), 'topomap_dir': 'route'})

    def test_diagnostics_are_opt_in(self):
        directory=self.root/'PlaceNav'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        (directory/'placenav').mkdir()
        (directory/'placenav/parser.py').write_text("parser.add_argument('--trace-navigation', action='store_true')")
        command,_=runner.navigation_command(dict(placenav_src=str(directory),topomap_dir='route',diagnostics=True))
        self.assertIn('--trace-navigation',command)
        command,_=runner.navigation_command(dict(placenav_src=str(directory),topomap_dir='route'))
        self.assertNotIn('--trace-navigation',command)

    def test_latest_checkout_receives_navigation_options_and_no_unsupported_trace(self):
        directory = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        spec = importlib.util.spec_from_file_location('selected_placenav_parser', directory/'placenav/parser.py')
        parser = importlib.util.module_from_spec(spec); spec.loader.exec_module(parser)
        params = dict(placenav_src=str(directory), topomap_dir='route with spaces',
                      start_node_idx=4, goal_node_idx=19, lookahead=2, nomad_seed=42,
                      close_threshold=5, target_waypoint_idx=6, device='cpu',
                      transition_model_window_lower=-2, transition_model_window_upper=4,
                      filter_delta=12, recompute_place_recognition_db=True, img_size=[96, 96],
                      motion_model='odometry', odom_topic='/vo/odom', odom_max_gap=.15, odom_noise=.2,
                      subgoal_update='nearest', pass_threshold=.6, pass_confirmations=3,
                      pass_local_mass=.9, final_goal_tolerance=.4,
                      global_frame='test_odom', robot_base_frame='test_base', diagnostics=True)
        command, _ = runner.navigation_command(params)
        self.assertNotIn('--trace-navigation', command)
        for option, expected in (('--path-adapter-source-frame', 'test_base'),
                                 ('--path-adapter-target-frame', 'test_odom')):
            self.assertEqual(command[command.index(option)+1], expected)
        required = ['--robot', 'whill', '--robot-config-path', 'robots.yaml',
                    '--model-config-path', 'models.yaml', '--topomap-base-dir', 'maps',
                    '--model-weight-dir', 'weights']
        with patch.object(sys, 'argv', ['navigate.py']+required+command[2:]):
            args = parser.parse_args()
        for key in ('topomap_dir', 'start_node_idx', 'goal_node_idx', 'lookahead', 'nomad_seed',
                    'close_threshold', 'target_waypoint_idx', 'device', 'transition_model_window_lower',
                    'transition_model_window_upper', 'filter_delta', 'recompute_place_recognition_db', 'img_size',
                    'motion_model', 'odom_topic', 'odom_max_gap', 'odom_noise',
                    'subgoal_update', 'pass_threshold', 'pass_confirmations',
                    'pass_local_mass', 'final_goal_tolerance'):
            self.assertEqual(getattr(args, key), params[key], key)

    def test_goal_evaluation_options_reach_parser_and_reject_invalid_ranges(self):
        directory = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        spec = importlib.util.spec_from_file_location('goal_options_parser', directory/'placenav/parser.py')
        parser = importlib.util.module_from_spec(spec);spec.loader.exec_module(parser)
        options = dict(goal_eval_position_tolerance=.4, goal_eval_yaw_tolerance=.2,
                       goal_eval_neighborhood_radius=3., subgoal_update='arrival',
                       goal_recovery_distance=5., goal_recovery_threshold=.9, local_pose=True)
        command, _ = runner.navigation_command(dict(placenav_src=str(directory), topomap_dir='route', **options))
        argv = command[2:] + ['--robot', 'whill', '--robot-config-path', '/tmp/robots.yaml',
            '--model-config-path', '/tmp/models.yaml', '--model-weight-dir', '/tmp/weights',
            '--topomap-base-dir', '/tmp/maps']
        with patch.object(sys, 'argv', ['navigate.py'] + argv):
            args = parser.parse_args()
        for key, value in options.items():
            self.assertEqual(getattr(args, key), value)
        for invalid in [dict(goal_eval_yaw_tolerance=4), dict(goal_eval_yaw_tolerance=float('nan')),
                        dict(goal_eval_position_tolerance=-1), dict(goal_eval_neighborhood_radius=.1)]:
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                runner.navigation_command(dict(placenav_src=str(directory), topomap_dir='route', **invalid))
        for invalid in [['--goal-eval-yaw-tolerance', '4'], ['--goal-eval-neighborhood-radius', '.1']]:
            with patch.object(sys, 'argv', ['navigate.py'] + argv + invalid), self.assertRaises(SystemExit):
                parser.parse_args()

    def test_wp_goal_pose_reaches_parser_and_requires_omnivla_with_local_pose(self):
        directory = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        spec = importlib.util.spec_from_file_location('selected_placenav_parser_goal_pose', directory/'placenav/parser.py')
        parser = importlib.util.module_from_spec(spec); spec.loader.exec_module(parser)
        base = dict(placenav_src=str(directory), topomap_dir='route', wp_model='omnivla_edge', img_size=[96, 96],
                    motion_model='odometry', local_pose=True, subgoal_update='arrival')
        required = ['--robot', 'whill', '--robot-config-path', 'robots.yaml',
                    '--model-config-path', 'models.yaml', '--topomap-base-dir', 'maps',
                    '--model-weight-dir', 'weights']
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                command, _ = runner.navigation_command(dict(base, wp_goal_pose=enabled))
                self.assertEqual('--wp-goal-pose' in command, enabled)
                with patch.object(sys, 'argv', ['navigate.py']+required+command[2:]):
                    self.assertEqual(parser.parse_args().wp_goal_pose, enabled)
        for invalid in ({'wp_model': 'nomad'}, {'local_pose': False}, {'wp_goal_pose': 'true'}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                runner.navigation_command(dict(base, **dict({'wp_goal_pose': True}, **invalid)))
        # The parser enforces the same rule for direct navigate.py / navigate_whill.sh use.
        with patch.object(sys, 'argv', ['navigate.py']+required+['--topomap-dir', 'route', '--wp-model', 'nomad',
                                                              '--local-pose', '--wp-goal-pose']), \
                patch('sys.stderr'), self.assertRaises(SystemExit):
            parser.parse_args()

    def test_invalid_motion_options_are_rejected(self):
        directory = self.root/'PlaceNav'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        for options in ({'motion_model': 'typo'}, {'odom_max_gap': 0},
                        {'odom_max_gap': float('nan')}, {'odom_noise': -1},
                        {'odom_noise': float('inf')}, {'subgoal_update': 'typo'},
                        {'pass_threshold': 1.1}, {'pass_confirmations': 0},
                        {'pass_local_mass': float('nan')}, {'final_goal_tolerance': -1},
                        {'subgoal_update': 'passage', 'motion_model': 'legacy'},
                        {'subgoal_update': 'passage', 'lookahead': 2},
                        {'subgoal_update': 'arrival', 'motion_model': 'legacy'},
                        {'subgoal_update': 'arrival', 'lookahead': 2},
                        {'goal_recovery_distance': 1}, {'goal_recovery_threshold': 0},
                        {'local_pose': True, 'motion_model': 'legacy'}, {'local_pose_reset': 'false'}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                runner.navigation_command(dict(placenav_src=str(directory), topomap_dir='route', **options))

    def test_yaml_degrees_are_converted_and_legacy_radians_take_priority(self):
        directory = self.root/'PlaceNav'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        base = dict(placenav_src=str(directory), topomap_dir='route')
        for options, expected in [({}, math.radians(20.)),
                                  ({'goal_eval_yaw_tolerance_deg': 35.}, math.radians(35.)),
                                  ({'goal_eval_yaw_tolerance_deg': 35.,
                                    'goal_eval_yaw_tolerance': .2}, .2)]:
            with self.subTest(options=options):
                command, _ = runner.navigation_command(dict(base, **options))
                self.assertEqual(float(command[command.index('--goal-eval-yaw-tolerance')+1]), expected)
        for degrees in [0., -20., 181., float('nan'), float('inf')]:
            with self.subTest(degrees=degrees), self.assertRaises(ValueError):
                runner.navigation_command(dict(base, goal_eval_yaw_tolerance_deg=degrees))

    def test_trace_capability_follows_selected_checkout(self):
        directory = self.root/'PlaceNav'; (directory/'placenav').mkdir(parents=True)
        parser = directory/'placenav/parser.py'
        parser.write_text("# --trace-navigation is not supported\nparser.add_argument('--device')")
        self.assertFalse(runner.supports_navigation_trace(directory))
        parser.write_text("parser.add_argument('--trace-navigation', action='store_true')")
        self.assertTrue(runner.supports_navigation_trace(directory))

    def test_invalid_image_size_rejected_before_starting_navigation(self):
        directory = self.root/'PlaceNav'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        for size in ('96,96', [96], [96, 0], [96.0, 96], [True, 96]):
            with self.assertRaises(ValueError):
                runner.navigation_command(dict(placenav_src=str(directory), topomap_dir='route', img_size=size))

    def test_whill_script_routes_frames_to_adapter_only(self):
        source = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        directory = self.root/'PlaceNav'; directory.mkdir()
        (directory/'navigate_whill.sh').write_text((source/'navigate_whill.sh').read_text())
        fake_bin = directory/'bin'; fake_bin.mkdir()
        python = fake_bin/'python3'
        python.write_text('#!'+sys.executable+'''\nimport json,sys,time
from pathlib import Path
Path('nav.json').write_text(json.dumps(sys.argv[1:]))
deadline=time.monotonic()+3
while not Path('adapter.json').exists() and time.monotonic()<deadline: time.sleep(.01)
''')
        python.chmod(0o755)
        adapter = directory/'run_gnm_path_adapter_whill.sh'
        adapter.write_text('#!'+sys.executable+'''\nimport json,sys
from pathlib import Path
Path('adapter.json').write_text(json.dumps(sys.argv[1:]))
''')
        adapter.chmod(0o755)
        command, cwd = runner.navigation_command(dict(placenav_src=str(directory), topomap_dir='route',
                                                      global_frame='test_odom', robot_base_frame='test_base',
                                                      visualization=False))
        env = dict(os.environ, PATH=str(fake_bin)+os.pathsep+os.environ['PATH'])
        subprocess.run(command, cwd=cwd, env=env, check=True, timeout=6)
        nav = json.loads((directory/'nav.json').read_text())
        args = json.loads((directory/'adapter.json').read_text())
        self.assertNotIn('--path-adapter-target-frame', nav)
        self.assertEqual(args[args.index('--source-frame')+1], 'test_base')
        self.assertEqual(args[args.index('--target-frame')+1], 'test_odom')
        self.assertEqual(args[args.index('--expected-waypoints')+1], '8')

    def test_whill_script_prefers_built_messages_over_an_old_shell(self):
        source = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        workspace = self.root/'ws'
        directory = workspace/'src/PlaceNav/src'
        (directory/'placenav').mkdir(parents=True)
        script = directory/'navigate_whill.sh'
        script.write_text((source/'navigate_whill.sh').read_text())
        old = self.root/'image_messages'
        built = workspace/'install/placenav_viz_msgs/lib/python3/dist-packages'
        for prefix, contents in [(old, '# Old image: no GoalEvaluation\n'),
                                 (built, 'class GoalEvaluation: pass\n')]:
            package = prefix/'placenav_viz_msgs'
            (package/'msg').mkdir(parents=True)
            (package/'__init__.py').touch()
            (package/'msg/__init__.py').write_text(contents)
        (directory/'placenav/navigate.py').write_text(
            'import json,placenav_viz_msgs.msg as messages\n'
            'from placenav_viz_msgs.msg import GoalEvaluation\n'
            'print(json.dumps(messages.__file__))\n')
        command = ['bash', str(script), '--topomap-dir', 'route',
                   '--no-visualization', '--no-path-adapter']
        env = dict(os.environ, PYTHONPATH=str(old))
        result = subprocess.run(command, cwd=directory, env=env, check=True,
                                capture_output=True, text=True, timeout=6)
        self.assertEqual(Path(json.loads(result.stdout)).resolve(),
                         (built/'placenav_viz_msgs/msg/__init__.py').resolve())

    def test_remote_navigation_splits_parrot_and_xavier_commands(self):
        directory = self.root/'PlaceNav'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        xavier = self.root/'xavier src'; xavier.mkdir(); (xavier/'navigate_whill.sh').touch()
        params = dict(placenav_src=str(directory), topomap_dir='route', wp_model='omnivla_edge',
                      pr_model='cosplace', start_node_idx=4, remote_navigation=True)
        parrot, _ = runner.navigation_command(params)
        self.assertIn('--remote-navigation', parrot)
        command, cwd = runner.xavier_navigation_command(params, str(xavier))
        self.assertEqual(cwd, xavier.resolve())
        self.assertEqual(command[1], str(xavier.resolve()/'navigate_whill.sh'))
        self.assertNotIn('--remote-navigation', command)
        self.assertIn('--no-path-adapter', command)
        self.assertIn('--no-visualization', command)
        # Everything that selects the route and models is identical on both machines.
        strip = {'--remote-navigation', '--no-path-adapter', '--no-visualization'}
        self.assertEqual([a for a in parrot[2:] if a not in strip], [a for a in command[2:] if a not in strip])
        with self.assertRaises(ValueError):  # parrot launch without remote_navigation:=true
            runner.xavier_navigation_command(dict(params, remote_navigation=False), str(xavier))

    def test_remote_odometry_options_reach_xavier_parser(self):
        directory = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        spec = importlib.util.spec_from_file_location('remote_placenav_parser', directory/'placenav/parser.py')
        parser = importlib.util.module_from_spec(spec); spec.loader.exec_module(parser)
        options = dict(
            placenav_src=str(directory), topomap_dir='odom route', wp_model='omnivla_edge',
            pr_model='cosplace', remote_navigation=True, motion_model='odometry',
            odom_topic='/test/vo', odom_max_gap=.15, odom_noise=.2, local_pose=True,
            pass_threshold=.6, pass_confirmations=3, pass_local_mass=.9,
            goal_recovery_distance=5., goal_recovery_threshold=.85,
            final_goal_tolerance=.4, goal_eval_position_tolerance=.3,
            goal_eval_yaw_tolerance_deg=35., goal_eval_neighborhood_radius=3.)
        required = ['--robot', 'whill', '--robot-config-path', '/unused/robots.yaml',
                    '--model-config-path', '/unused/models.yaml',
                    '--model-weight-dir', '/unused/weights', '--topomap-base-dir', '/unused/maps']
        for mode in ('nearest', 'passage', 'arrival'):
            with self.subTest(mode=mode):
                params = dict(options, subgoal_update=mode)
                parrot, _ = runner.navigation_command(params)
                xavier, _ = runner.xavier_navigation_command(params, str(directory))
                strip = {'--remote-navigation', '--no-path-adapter', '--no-visualization'}
                self.assertEqual([a for a in parrot[2:] if a not in strip],
                                 [a for a in xavier[2:] if a not in strip])
                with patch.object(sys, 'argv', ['navigate.py']+required+xavier[2:]):
                    args = parser.parse_args()
                for name in ('topomap_dir', 'wp_model', 'pr_model', 'motion_model', 'odom_topic',
                             'odom_max_gap', 'odom_noise', 'local_pose', 'pass_threshold',
                             'pass_confirmations', 'pass_local_mass', 'goal_recovery_distance',
                             'goal_recovery_threshold', 'final_goal_tolerance',
                             'goal_eval_position_tolerance', 'goal_eval_neighborhood_radius'):
                    self.assertEqual(getattr(args, name), options[name], name)
                self.assertEqual(args.subgoal_update, mode)
                self.assertAlmostEqual(args.goal_eval_yaw_tolerance, math.radians(35.))

    def test_remote_whill_script_only_runs_adapter_and_preserves_local_db(self):
        source = Path(os.environ.get('PLACENAV_TEST_SRC', Path(__file__).resolve().parents[2]))
        directory = self.root/'remote'; directory.mkdir()
        script = directory/'navigate_whill.sh'
        script.write_text((source/'navigate_whill.sh').read_text())
        ready = directory/'topomaps/images/route/global-feats-cosplace.h5.ready'
        ready.parent.mkdir(parents=True); ready.write_text('local database untouched')
        fake_bin = directory/'bin'; fake_bin.mkdir()
        python = fake_bin/'python3'
        python.write_text('#!'+sys.executable+'\nfrom pathlib import Path\n'
                          'Path("unexpected_inference").touch()\nraise SystemExit(97)\n')
        python.chmod(0o755)
        adapter = directory/'run_gnm_path_adapter_whill.sh'
        adapter.write_text('#!'+sys.executable+'\nimport json,sys\nfrom pathlib import Path\n'
                           'Path("adapter.json").write_text(json.dumps(sys.argv[1:]))\n')
        adapter.chmod(0o755)
        command = ['bash', str(script), '--remote-navigation', '--topomap-dir', 'route',
                   '--wp-model', 'omnivla_edge', '--pr-model', 'cosplace',
                   '--path-adapter-source-frame', 'test_base', '--path-adapter-target-frame', 'test_odom']
        env = dict(os.environ, PATH=str(fake_bin)+os.pathsep+os.environ['PATH'])
        subprocess.run(command, cwd=directory, env=env, check=True, timeout=6, capture_output=True)
        self.assertFalse((directory/'unexpected_inference').exists())
        self.assertEqual(ready.read_text(), 'local database untouched')
        args = json.loads((directory/'adapter.json').read_text())
        for flag, value in (('--expected-waypoints', '8'), ('--source-frame', 'test_base'),
                            ('--target-frame', 'test_odom')):
            self.assertEqual(args[args.index(flag)+1], value)
        rejected = subprocess.run(command+['--no-path-adapter'], cwd=directory, env=env,
                                  capture_output=True, text=True, timeout=6)
        self.assertEqual(rejected.returncode, 2)

    def test_xavier_runner_stops_on_parrot_disappearance_or_restart(self):
        directory = self.root/'xavier'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        for replacement in (None, 'http://127.0.0.1:40202/'):
            with self.subTest(replacement=replacement):
                ros = Mock()
                ros.is_shutdown.return_value = False
                ros.has_param.return_value = True
                ros.get_param.return_value = dict(
                    placenav_src='/parrot/src', topomap_dir='route', remote_navigation=True,
                    motion_model='odometry', subgoal_update='arrival', local_pose=True)
                graph = Mock()
                graph.Master.return_value.lookupNode.side_effect = [
                    'http://127.0.0.1:40101/', 'http://127.0.0.1:40101/', replacement]
                process = Mock(); process.poll.return_value = None
                with patch.dict(sys.modules, rospy=ros, rosgraph=graph), \
                        patch.object(runner.subprocess, 'Popen', return_value=process) as start, \
                        patch.object(runner, 'stop_process') as stop:
                    status = runner.xavier_main(['--xavier', '--placenav-src', str(directory)])
                self.assertEqual(status, 0)
                self.assertTrue(start.call_args.kwargs['start_new_session'])
                command = start.call_args.args[0]
                self.assertIn('--local-pose', command)
                self.assertIn('--no-path-adapter', command)
                self.assertNotIn('--remote-navigation', command)
                stop.assert_called_once_with(process)

    def test_xavier_runner_propagates_navigation_exit_and_cleans_children(self):
        directory = self.root/'xavier'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        for child_status, expected in ((0, 0), (7, 1)):
            with self.subTest(child_status=child_status):
                ros = Mock()
                ros.is_shutdown.return_value = False
                ros.has_param.return_value = True
                ros.get_param.return_value = dict(topomap_dir='route', remote_navigation=True)
                graph = Mock()
                graph.Master.return_value.lookupNode.return_value = 'http://127.0.0.1:40101/'
                process = Mock(); process.poll.return_value = child_status
                with patch.dict(sys.modules, rospy=ros, rosgraph=graph), \
                        patch.object(runner.subprocess, 'Popen', return_value=process), \
                        patch.object(runner, 'stop_process') as stop:
                    status = runner.xavier_main(['--xavier', '--placenav-src', str(directory)])
                self.assertEqual(status, expected)
                stop.assert_called_once_with(process)

    def test_xavier_runner_rejects_nonremote_session_and_wait_timeout(self):
        directory = self.root/'xavier'; directory.mkdir(); (directory/'navigate_whill.sh').touch()
        ros = Mock()
        ros.is_shutdown.return_value = False
        ros.has_param.return_value = True
        ros.get_param.return_value = dict(topomap_dir='route', remote_navigation=False)
        graph = Mock()
        graph.Master.return_value.lookupNode.return_value = 'http://127.0.0.1:40101/'
        with patch.dict(sys.modules, rospy=ros, rosgraph=graph), \
                patch.object(runner.subprocess, 'Popen') as start:
            self.assertEqual(runner.xavier_main(['--xavier', '--placenav-src', str(directory)]), 1)
            start.assert_not_called()
            graph.Master.return_value.lookupNode.return_value = None
            with patch.object(runner.time, 'monotonic', side_effect=[10., 11.]):
                self.assertEqual(runner.xavier_main(['--xavier', '--wait', '0']), 1)
            start.assert_not_called()

    def assert_not_running(self, pid):
        path = Path('/proc')/str(pid)/'stat'
        self.assertTrue(not path.exists() or path.read_text().split()[2] == 'Z')

    def test_cleanup_stops_descendants_after_script_exits(self):
        child_pid = self.root/'child.pid'
        code = '''import subprocess,sys,time
from pathlib import Path
ready=sys.argv[1]+'.ready'
child='import signal,time,sys; from pathlib import Path; signal.signal(signal.SIGINT,signal.SIG_IGN); Path(sys.argv[1]).write_text("ready"); time.sleep(300)'
p=subprocess.Popen([sys.executable,'-c',child,ready])
deadline=time.monotonic()+4
while not Path(ready).exists():
 if time.monotonic()>deadline: raise RuntimeError('child did not start')
 time.sleep(.01)
open(sys.argv[1],'w').write(str(p.pid))
'''
        process = subprocess.Popen(['python3', '-c', code, str(child_pid)], start_new_session=True)
        try:
            process.wait(timeout=5)
            self.assertTrue(child_pid.exists())
            runner.stop_process(process)
            self.assert_not_running(int(child_pid.read_text()))
        finally:
            runner.stop_process(process)


if __name__ == '__main__': unittest.main()
