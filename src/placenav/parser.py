'''
Argument parser for the placenav node
(navigate.py)
'''

import argparse
import math
from pathlib import Path


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def nonnegative_float(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError('must be finite and nonnegative')
    return number


def probability(value):
    number = positive_float(value)
    if number > 1:
        raise argparse.ArgumentTypeError('must be in (0, 1]')
    return number


def positive_int(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError('must be a positive integer')
    return number

def parse_args():
    parser = argparse.ArgumentParser(
        description=f"PlaceNav node for topological image navigation"
    )
    parser.add_argument(
        "--robot",
        type=str,
        help="robot name",
        required=True,
    )
    parser.add_argument(
        "--img-size",
        "-i",
        default=None,
        type=int,
        nargs=2,
        help="waypoint image size; default is selected model's image_size (not VPR size)",
    )
    parser.add_argument(
        "--topomap-dir",
        "-d",
        type=str,
        help="name of the topomap directory",
        required=True,
    )
    parser.add_argument(
        "--wp-model",
        default="gnm_large",
        type=str,
        choices=["gnm_large", "vint", "nomad", "omnivla_edge"],
        help="waypoint model (default: gnm_large); prepare ViNT/NoMaD/OmniVLA-edge weights first",
    )
    parser.add_argument(
        "--pr-model",
        default="cosplace",
        type=str,
        choices=["cosplace", "megaloc", "salad", "anyloc"],
        help="VPR backend (prepare new models with prepare_vpr.py first; default: cosplace)",
    )
    parser.add_argument("--nomad-seed", type=int, default=0,
                        help="per-inference NoMaD diffusion seed (default: 0)")
    parser.add_argument(
        "--close-threshold",
        "-t",
        default=3,
        type=int,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--window-radius",
        "-r",
        default=2,
        type=int,
        help="""temporal number of topomap nodes to look at in the topopmap for
        localization when using sliding window filter (default: 2)""",
    )
    parser.add_argument(
        "--target-waypoint-idx",
        "-w",
        default=2, # close waypoints exihibit straight line motion (the middle waypoint is a good default)
        type=int,
        help=f"""index of the waypoint used for navigation (between 0 and 4 or 
        how many waypoints your model predicts) (default: 2)""",
    )
    parser.add_argument(
        "--goal-node-idx",
        "-g",
        default=-1,
        type=int,
        help="""goal node index in the topomap (if -1, then the goal node is 
        the last node in the topomap) (default: -1)""",
    )
    parser.add_argument(
        "--start-node-idx",
        "-s",
        default=0,
        type=int,
        help="""start node index in the topomap (if 0, then the start node is 
        the first node in the topomap) (default: 0)""",
    )
    parser.add_argument(
        "--filter-mode",
        default="bayesian",
        type=str,
        help="Filter mode for choosing the subgoal: bayesian (default) or sliding_window",
    )
    parser.add_argument(
        "--subgoal-mode",
        default='place_recognition',
        type=str,
        help="""If the subgoal should be chosen using GNM image temporal distance prediction
         or place recognition (place_recognition | temporal_distance) (default: place_recognition)"""
    )
    parser.add_argument(
        "--lookahead",
        default=1,
        type=int,
        help="""Number of nodes ahead from current node to consider for subgoal
            when choosing subgoals with place recognition (default: 1)""",
    )
    parser.add_argument(
        "--recompute-place-recognition-db",
        action="store_true",
        default=False
    )
    parser.add_argument(
        "--device",
        default="cuda",
        type=str,
        help="Device to use for inference (cuda | cpu) (default: cuda)",
    )

    # Bayesian filter params
    parser.add_argument('--motion-model', choices=['legacy', 'odometry'], default='odometry',
                        help='Bayesian prediction: fixed node window or live odometry')
    parser.add_argument('--odom-topic', help='nav_msgs/Odometry input; default: robot config')
    parser.add_argument('--odom-max-gap', type=positive_float, default=0.2,
                        help='Maximum bracketing odometry interval in seconds')
    parser.add_argument('--odom-noise', type=nonnegative_float, default=0.1,
                        help='Along-route process std in meters per sqrt(meter travelled)')
    parser.add_argument('--local-pose', action='store_true',
                        help='Publish diagnostic-only local 2D pose and relative image goal')
    parser.add_argument('--wp-goal-pose', action='store_true',
                        help='Experimental, OmniVLA-edge only: also feed the --local-pose relative '
                             'subgoal pose (upstream modality 5, pose and image). Falls back to the '
                             'image-only goal while the local pose is not valid')
    parser.add_argument('--heading-correction', default='none',
                        help='Measure the heading at subgoal switches and set the --local-pose heading: '
                             "'none' (default) or an estimator from placenav.heading_correction "
                             "(e.g. 'sift_shift': horizontal shift of image matches)")
    parser.add_argument('--camera-hfov-deg', type=positive_float,
                        help='Equidistant horizontal field of view for --heading-correction '
                             '(default: camera_hfov_deg in the robot config)')
    parser.add_argument('--nav-role', choices=['full', 'localization', 'waypoint'], default='full',
                        help="'full' (default): everything in one process. Split over two machines: "
                             "'localization' runs VPR, the filter, subgoal selection, local pose and "
                             "heading correction and publishes /toponav/subgoal_command; 'waypoint' "
                             "runs only the waypoint model toward that subgoal")
    parser.add_argument('--subgoal-command-timeout', type=positive_float, default=0.6,
                        help="--nav-role waypoint: do not publish waypoints when the latest subgoal "
                             "command is further than this from the camera frame, in seconds")
    resets = parser.add_mutually_exclusive_group()
    resets.add_argument('--local-pose-reset', dest='local_pose_reset', action='store_true', default=True,
                        help='Enable local XY anchors at goal switches and recovery (default)')
    resets.add_argument('--no-local-pose-reset', dest='local_pose_reset', action='store_false',
                        help='Keep local pose output but disable XY resets in every subgoal mode')
    parser.add_argument('--subgoal-update', choices=['nearest', 'passage', 'arrival', 'arrival_turn', 'arrival_turn_step'], default='nearest',
                        help='Nearest+lookahead, retained passage, position arrival, or arrival with relative odometry turn')
    parser.add_argument('--turn-check-start-deg', type=positive_float, default=30.)
    parser.add_argument('--turn-required-ratio', type=probability, default=0.25)
    parser.add_argument('--pass-threshold', type=probability, default=0.5)
    parser.add_argument('--pass-confirmations', type=positive_int, default=2,
                        help='Fresh-image confirmations for passage, recovery and final stopping; '
                             'intermediate arrival uses one posterior update')
    parser.add_argument('--pass-local-mass', type=probability, default=0.8,
                        help='Required posterior mass in the metric goal neighborhood')
    parser.add_argument('--goal-recovery-distance', type=positive_float, default=4.0,
                        help='Backward discrepancy in route meters before goal recovery')
    parser.add_argument('--goal-recovery-threshold', type=probability, default=0.8)
    parser.add_argument('--final-goal-tolerance', type=nonnegative_float, default=0.5,
                        help='Final goal progress tolerance in meters (all odometry subgoal modes)')
    parser.add_argument('--goal-eval-position-tolerance', type=positive_float, default=0.5,
                        help='Arrival interval half-width in route meters')
    parser.add_argument('--goal-eval-yaw-tolerance', type=positive_float, default=math.radians(20),
                        help='Heading diagnostic tolerance in radians, at most pi; does not gate arrival')
    parser.add_argument('--goal-eval-neighborhood-radius', type=positive_float, default=2.0,
                        help='Goal neighborhood and forward recovery distance in route meters')
    parser.add_argument(
        "--transition-model-window-lower",
        default=-1,
        type=int,
        help="""Lower window of the transition model (default: -1)""",
    )
    parser.add_argument(
        "--transition-model-window-upper",
        default=2,
        type=int,
        help="""Upper window of the transition model (default: 2)""",
    )
    parser.add_argument(
        "--filter-delta",
        default=10,
        type=int,
        help="""Filter delta (default: 10)""",
    )

    # Config file paths
    parser.add_argument(
        "--robot-config-path",
        type=Path,
        help="path to config of the robot to control",
        required=True,
    )
    parser.add_argument(
        "--model-config-path",
        type=Path,
        help="path to config of the models",
        required=True,
    )
    parser.add_argument(
        "--topomap-base-dir",
        type=Path,
        help="path to topomap directory",
        required=True,
    )
    parser.add_argument(
        "--model-weight-dir",
        type=Path,
        help="path to model weights directory",
        required=True,
    )


    args, _ = parser.parse_known_args()
    if args.local_pose and args.motion_model != 'odometry':
        parser.error('--local-pose requires --motion-model odometry')
    if args.wp_goal_pose and (args.wp_model != 'omnivla_edge' or not args.local_pose):
        parser.error('--wp-goal-pose requires --wp-model omnivla_edge and --local-pose')
    if args.heading_correction != 'none':
        from placenav.heading_correction import heading_estimator_names
        if args.heading_correction not in heading_estimator_names():
            parser.error(f"--heading-correction must be 'none' or one of {heading_estimator_names()}")
        if not args.local_pose:
            parser.error('--heading-correction requires --local-pose')
    if args.goal_eval_yaw_tolerance > math.pi:
        parser.error('--goal-eval-yaw-tolerance must not exceed pi')
    if args.goal_eval_neighborhood_radius < args.goal_eval_position_tolerance:
        parser.error('--goal-eval-neighborhood-radius must be at least the position tolerance')
    if args.goal_recovery_distance < args.goal_eval_neighborhood_radius:
        parser.error('--goal-recovery-distance must cover the goal neighborhood')
    if args.motion_model == 'odometry' and (
            args.filter_mode != 'bayesian' or args.subgoal_mode != 'place_recognition'):
        parser.error('--motion-model odometry requires --filter-mode bayesian '
                     '--subgoal-mode place_recognition')
    if args.nav_role != 'full' and args.subgoal_mode != 'place_recognition':
        parser.error('--nav-role localization/waypoint requires --subgoal-mode place_recognition')
    if args.subgoal_update != 'nearest' and (
            args.motion_model != 'odometry' or args.lookahead != 1):
        parser.error('--subgoal-update passage/arrival requires --motion-model odometry --lookahead 1')
    return args
