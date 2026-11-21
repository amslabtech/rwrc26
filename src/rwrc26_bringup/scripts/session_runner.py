#!/usr/bin/env python3
"""ROS launch adapter for the existing navigation script.

navigate.py takes command-line options, while navigation.launch keeps its settings as ROS
parameters (placenav.yaml plus explicit launch overrides). This node turns those parameters
into the navigate_whill.sh command line and stops the whole process group on shutdown.
Only main() and xavier_main() import rospy so command construction and process
cleanup can be tested without ROS, model loading, a camera, or robot command publishers.

On the Xavier (split inference), `session_runner.py --xavier` reads the parameters that
navigation.launch put under /placenav_runner on parrot and starts navigate.py with the
same command line, so both machines can never disagree on route, models or indices.
remote_role selects what runs there: 'all' (navigate.py; parrot runs only the path adapter)
or 'waypoint' (only the waypoint model; parrot runs localization, the path adapter and the GUI).
"""
import ast
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def supports_navigation_trace(directory):
    """Inspect the selected checkout without importing ROS, Torch or models."""
    parser = Path(directory)/'placenav/parser.py'
    if not parser.is_file():
        return False
    tree = ast.parse(parser.read_text(encoding='utf-8'))
    return any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
               and node.func.attr == 'add_argument'
               and any(isinstance(arg, ast.Constant) and arg.value == '--trace-navigation'
                       for arg in node.args) for node in ast.walk(tree))


def navigation_command(params):
    directory = Path(params['placenav_src']).expanduser().resolve()
    script = directory/'navigate_whill.sh'
    if not script.is_file():
        raise ValueError('navigate_whill.sh not found: '+str(script))
    radius = int(params.get('window_radius', 3))
    if radius < 1:
        raise ValueError('window_radius must be positive')
    command = ['bash', str(script),
               '--topomap-dir', str(params['topomap_dir']),
               '--wp-model', str(params.get('wp_model', 'nomad')),
               '--pr-model', str(params.get('pr_model', 'salad')),
               '--subgoal-mode', str(params.get('subgoal_mode', 'place_recognition')),
               '--filter-mode', str(params.get('filter_mode', 'bayesian')),
               '--window-radius', str(radius)]
    integers = {'start_node_idx': 0, 'goal_node_idx': -1, 'lookahead': 1,
                'close_threshold': 3, 'target_waypoint_idx': 2, 'nomad_seed': 0,
                'transition_model_window_lower': -1, 'transition_model_window_upper': 2,
                'filter_delta': 10}
    for name, default in integers.items():
        command += ['--'+name.replace('_', '-'), str(int(params.get(name, default)))]
    command += ['--device', str(params.get('device', 'cuda'))]
    motion = str(params.get('motion_model', 'odometry'))
    if motion not in ('legacy', 'odometry'):
        raise ValueError('motion_model must be legacy or odometry')
    gap = float(params.get('odom_max_gap', 0.2))
    noise = float(params.get('odom_noise', 0.1))
    if not math.isfinite(gap) or gap <= 0 or not math.isfinite(noise) or noise < 0:
        raise ValueError('odom_max_gap must be positive and odom_noise nonnegative')
    command += ['--motion-model', motion,
                '--odom-topic', str(params.get('odom_topic', '/odom')),
                '--odom-max-gap', str(gap), '--odom-noise', str(noise)]
    if params.get('local_pose', False):
        if motion != 'odometry':
            raise ValueError('local_pose requires odometry')
        command.append('--local-pose')
    goal_pose = params.get('wp_goal_pose', False)
    if not isinstance(goal_pose, bool):
        raise ValueError('wp_goal_pose must be a YAML boolean')
    if goal_pose:
        if str(params.get('wp_model', 'nomad')) != 'omnivla_edge' or not params.get('local_pose', False):
            raise ValueError('wp_goal_pose requires wp_model omnivla_edge and local_pose')
        command.append('--wp-goal-pose')
    heading = str(params.get('heading_correction', 'none'))
    if not heading or any(c.isspace() for c in heading):
        raise ValueError('heading_correction must be none or an estimator name')
    if heading != 'none':
        if not params.get('local_pose', False):
            raise ValueError('heading_correction requires local_pose')
        command += ['--heading-correction', heading]
        if 'camera_hfov_deg' in params:
            hfov = float(params['camera_hfov_deg'])
            if not math.isfinite(hfov) or not 0 < hfov < 360:
                raise ValueError('camera_hfov_deg must be in (0, 360)')
            command += ['--camera-hfov-deg', str(hfov)]
    reset = params.get('local_pose_reset', True)
    if not isinstance(reset, bool):
        raise ValueError('local_pose_reset must be a YAML boolean')
    command.append('--local-pose-reset' if reset else '--no-local-pose-reset')
    update = str(params.get('subgoal_update', 'nearest'))
    if update not in ('nearest', 'passage', 'arrival'):
        raise ValueError('subgoal_update must be nearest, passage or arrival')
    if update != 'nearest' and (motion != 'odometry' or int(params.get('lookahead', 1)) != 1):
        raise ValueError('passage/arrival require odometry and lookahead=1')
    command += ['--subgoal-update', update]
    for name, default in [('pass_threshold', .5), ('pass_local_mass', .8),
                          ('goal_recovery_threshold', .8),
                          ('final_goal_tolerance', .5)]:
        value = float(params.get(name, default))
        if not math.isfinite(value) or (value < 0 if name == 'final_goal_tolerance' else not 0 < value <= 1):
            raise ValueError('Invalid '+name)
        command += ['--'+name.replace('_', '-'), str(value)]
    count = int(params.get('pass_confirmations', 2))
    if count < 1:
        raise ValueError('pass_confirmations must be positive')
    command += ['--pass-confirmations', str(count)]
    # YAML uses readable degrees. Preserve the existing explicit radian override.
    if 'goal_eval_yaw_tolerance' in params:
        yaw_tolerance = float(params['goal_eval_yaw_tolerance'])
    else:
        yaw_tolerance = math.radians(float(params.get('goal_eval_yaw_tolerance_deg', 20.)))
    evaluation = dict(goal_eval_position_tolerance=float(params.get('goal_eval_position_tolerance', .5)),
                      goal_eval_yaw_tolerance=yaw_tolerance,
                      goal_eval_neighborhood_radius=float(params.get('goal_eval_neighborhood_radius', 2.)))
    if (any(not math.isfinite(v) or v <= 0 for v in evaluation.values()) or
            evaluation['goal_eval_yaw_tolerance'] > math.pi or
            evaluation['goal_eval_neighborhood_radius'] < evaluation['goal_eval_position_tolerance']):
        raise ValueError('Invalid goal evaluation tolerances')
    for name, value in evaluation.items():
        command += ['--'+name.replace('_', '-'), str(value)]
    recovery_distance = float(params.get('goal_recovery_distance', 4.))
    if not math.isfinite(recovery_distance) or recovery_distance < evaluation['goal_eval_neighborhood_radius']:
        raise ValueError('goal_recovery_distance must cover the goal neighborhood')
    command += ['--goal-recovery-distance', str(recovery_distance)]
    size = params.get('img_size', [])
    if not isinstance(size, list) or (size and (len(size) != 2 or any(
            type(value) is not int or value <= 0 for value in size))):
        raise ValueError('img_size must be [] or [positive width, positive height]')
    if size:
        command += ['--img-size']+[str(value) for value in size]
    if params.get('recompute_place_recognition_db', False):
        command.append('--recompute-place-recognition-db')
    command += ['--path-adapter-source-frame', str(params.get('robot_base_frame', 'base_link')),
                '--path-adapter-target-frame', str(params.get('global_frame', 'odom'))]
    if not params.get('visualization', True):
        command.append('--no-visualization')
    if params.get('diagnostics', False) and supports_navigation_trace(directory):
        command.append('--trace-navigation')
    role = remote_role(params)
    if role == 'waypoint':
        # parrot: VPR, filter, local pose, heading correction; the Xavier only predicts waypoints
        command += ['--nav-role', 'localization']
    elif role == 'all':
        command.append('--remote-navigation')  # parrot: path adapter only
    # navigate_whill.sh starts the GNM path adapter itself. Do not duplicate it.
    return command, directory


def remote_role(params):
    """None (all on this machine), 'all' or 'waypoint': what navigate.py runs on the Xavier."""
    remote = params.get('remote_navigation', False)
    if not isinstance(remote, bool):
        raise ValueError('remote_navigation must be a YAML boolean')
    role = str(params.get('remote_role', 'all'))
    if role not in ('all', 'waypoint'):
        raise ValueError('remote_role must be all or waypoint')
    return role if remote else None


def xavier_navigation_command(params, placenav_src):
    """navigate.py alone, with parrot's launch parameters, from this machine's checkout."""
    role = remote_role(params)
    if role is None:
        raise ValueError('navigation.launch on parrot was not started with remote_navigation:=true')
    local = dict(params, placenav_src=placenav_src, remote_navigation=False, visualization=False)
    command, directory = navigation_command(local)
    command.append('--no-path-adapter')
    if role == 'waypoint':
        timeout = float(params.get('subgoal_command_timeout', .6))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('subgoal_command_timeout must be positive')
        command += ['--nav-role', 'waypoint', '--subgoal-command-timeout', str(timeout)]
    return command, directory


def stop_process(process):
    """Stop only the process group created by this runner, including children."""
    for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        try:
            # Do this even when the script exited but its children remain.
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # The group can still contain descendants after the group leader exits.
        deadline = time.monotonic()+min(timeout, .5)
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.025)
    process.wait()


def main():
    import rospy
    rospy.init_node('session_runner')
    try:
        command, directory = navigation_command(rospy.get_param('~'))
    except (KeyError, TypeError, ValueError, OSError, SyntaxError) as error:
        rospy.logfatal('Session configuration: %s', error)
        return 1
    if rospy.get_param('~diagnostics', False) and '--trace-navigation' not in command:
        rospy.logwarn('Selected PlaceNav has no --trace-navigation support; recording available ROS diagnostics only.')
    rospy.loginfo('Starting navigation in %s', directory)
    try:
        process = subprocess.Popen(command, cwd=str(directory), start_new_session=True)
    except OSError as error:
        rospy.logfatal('Could not start navigation: %s', error)
        return 1
    lock = threading.Lock()
    def cleanup():
        with lock:
            stop_process(process)
    rospy.on_shutdown(cleanup)
    try:
        while not rospy.is_shutdown():
            status = process.poll()
            if status is not None:
                if status:
                    rospy.logerr('navigation exited with status %s', status)
                else:
                    rospy.loginfo('navigation finished')
                return 0 if status == 0 else 1
            time.sleep(.1)  # Wall time: shutdown works even when ROS time pauses.
        return 0
    finally:
        cleanup()


def xavier_main(argv):
    """Run navigate.py for the parrot launch until that launch (its placenav_runner) ends."""
    import argparse
    import rosgraph
    import rospy
    parser = argparse.ArgumentParser(description=xavier_main.__doc__)
    parser.add_argument('--xavier', action='store_true')
    parser.add_argument('--placenav-src', default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument('--runner', default='/placenav_runner', help='parrot node holding the parameters')
    parser.add_argument('--wait', type=float, default=120.0, help='seconds to wait for the parrot launch')
    args = parser.parse_args(argv)
    rospy.init_node('placenav_xavier_runner')
    master = rosgraph.Master('/placenav_xavier_runner')

    def runner_uri():
        try:
            return master.lookupNode(args.runner)
        except Exception:  # noqa: BLE001 - not registered (or master gone) = parrot launch ended
            return None

    deadline = time.monotonic()+args.wait
    while not (runner_uri() and rospy.has_param(args.runner+'/remote_navigation')):
        if rospy.is_shutdown() or time.monotonic() > deadline:
            rospy.logfatal('No %s with remote_navigation on the master. Start navigation.launch '
                           'remote_navigation:=true on parrot first.', args.runner)
            return 1
        rospy.loginfo_throttle(10, 'Waiting for navigation.launch remote_navigation:=true on parrot...')
        time.sleep(.5)
    session = runner_uri()  # a restarted launch registers again under a new URI
    try:
        command, directory = xavier_navigation_command(rospy.get_param(args.runner), args.placenav_src)
    except (KeyError, TypeError, ValueError, OSError, SyntaxError) as error:
        rospy.logfatal('Session configuration: %s', error)
        return 1
    rospy.loginfo('Starting navigation in %s: %s', directory, ' '.join(command[2:]))
    process = subprocess.Popen(command, cwd=str(directory), start_new_session=True)
    lock = threading.Lock()
    def cleanup():
        with lock:
            stop_process(process)
    rospy.on_shutdown(cleanup)
    try:
        while not rospy.is_shutdown():
            status = process.poll()
            if status is not None:
                rospy.logerr('navigation exited with status %s', status) if status else rospy.loginfo('navigation finished')
                return 0 if status == 0 else 1
            if runner_uri() != session:
                rospy.loginfo('%s ended or restarted on parrot; stopping navigation here', args.runner)
                return 0
            time.sleep(.5)
        return 0
    finally:
        cleanup()


if __name__ == '__main__':
    import sys
    if '--xavier' in sys.argv[1:]:
        raise SystemExit(xavier_main(sys.argv[1:]))
    raise SystemExit(main())
