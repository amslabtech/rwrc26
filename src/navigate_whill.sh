#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${DISPLAY:-}" ]]; then
    for x_socket in /tmp/.X11-unix/X*; do
        if [[ -S "${x_socket}" ]]; then
            export DISPLAY=":${x_socket##*/X}"
            echo "DISPLAY was unset; using local X11 display ${DISPLAY}"
            break
        fi
    done
fi
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# An already-open shell may still resolve the image's older message package.
# Prefer the workspace build for navigation and its adapter/visualizer children.
MESSAGE_PYTHON_DIR="${SCRIPT_DIR}/../../../install/placenav_viz_msgs/lib/python3/dist-packages"
if [[ -d "${MESSAGE_PYTHON_DIR}/placenav_viz_msgs" ]]; then
    export PYTHONPATH="${MESSAGE_PYTHON_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
fi
ROBOT_CONFIG="${SCRIPT_DIR}/config/robots.yaml"
MODEL_CONFIG="${SCRIPT_DIR}/config/models.yaml"
MODEL_WEIGHT_DIR="${SCRIPT_DIR}/model_weights"
TOPOMAP_BASE_DIR="${SCRIPT_DIR}/topomaps/images"

usage()
{
    printf '%s\n' \
      'Usage: ./navigate_whill.sh --topomap-dir ROUTE [PlaceNav options]' \
      '  --pr-model cosplace|megaloc|salad|anyloc (default: cosplace)' \
      '  --wp-model gnm_large|vint|nomad|omnivla_edge (default: gnm_large)' \
      '  --motion-model legacy|odometry (default: odometry; requires node_odometry.json)' \
      '  --subgoal-update nearest|passage|arrival (default: nearest; passage includes current-node recovery)' \
      '  --goal-eval-position-tolerance METERS --goal-eval-yaw-tolerance RADIANS' \
      '  --goal-eval-neighborhood-radius METERS (diagnostics only; defaults: 0.5, 0.3491, 2.0)' \
      '' \
      'Starts inference and the GUI visualization. Use --no-visualization to suppress the GUI.' \
      'Starts the GNM Path adapter. Use --no-path-adapter to suppress it.' \
      '  --remote-navigation: start only the path adapter; navigate.py runs on another' \
      '    machine (docker/xavier/navigate_xavier.sh). No GUI: its database is over there.' \
      '  --path-adapter-source-frame FRAME (default: base_link)' \
      '  --path-adapter-target-frame FRAME (default: odom)' \
      'It never starts the PD controller or robot drivers.'
}

if [[ $# -eq 0 ]]; then
    usage
    exit 2
fi

TOPOMAP_DIR=""
PR_MODEL="cosplace"
WP_MODEL="gnm_large"
SHOW_VISUALIZATION=1
START_PATH_ADAPTER=1
REMOTE_NAVIGATION=0
PATH_SOURCE_FRAME=base_link
PATH_TARGET_FRAME=odom
NAV_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --wp-model)
            WP_MODEL="$2"
            NAV_ARGS+=("$1" "$2")
            shift 2
            ;;
        --topomap-dir)
            TOPOMAP_DIR="$2"
            NAV_ARGS+=("$1" "$2")
            shift 2
            ;;
        --pr-model)
            PR_MODEL="$2"
            NAV_ARGS+=("$1" "$2")
            shift 2
            ;;
        --no-visualization)
            SHOW_VISUALIZATION=0
            shift
            ;;
        --no-path-adapter)
            START_PATH_ADAPTER=0
            shift
            ;;
        --remote-navigation)
            REMOTE_NAVIGATION=1
            shift
            ;;
        --path-adapter-source-frame)
            PATH_SOURCE_FRAME="$2"
            shift 2
            ;;
        --path-adapter-target-frame)
            PATH_TARGET_FRAME="$2"
            shift 2
            ;;
        *)
            NAV_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ -z "${TOPOMAP_DIR}" ]]; then
    echo "--topomap-dir is required" >&2
    exit 2
fi

if [[ "${REMOTE_NAVIGATION}" -eq 1 && "${START_PATH_ADAPTER}" -eq 0 ]]; then
    echo "--remote-navigation runs only the path adapter; do not combine it with --no-path-adapter" >&2
    exit 2
fi

DB_PATH="${TOPOMAP_BASE_DIR}/${TOPOMAP_DIR}/global-feats-${PR_MODEL}.h5"
DB_READY_PATH="${DB_PATH}.ready"
NAV_PID=""
VIZ_PID=""
if [[ "${REMOTE_NAVIGATION}" -eq 1 ]]; then
    SHOW_VISUALIZATION=0
    echo "Remote navigation: navigate.py is expected on the Xavier (docker/xavier/navigate_xavier.sh)."
else
    rm -f "${DB_READY_PATH}"
    python3 "${SCRIPT_DIR}/placenav/navigate.py" \
        --robot whill \
        --robot-config-path "${ROBOT_CONFIG}" \
        --topomap-base-dir "${TOPOMAP_BASE_DIR}" \
        --model-weight-dir "${MODEL_WEIGHT_DIR}" \
        --model-config-path "${MODEL_CONFIG}" \
        "${NAV_ARGS[@]}" &
    NAV_PID=$!
fi
PATH_ADAPTER_PID=""

if [[ "${START_PATH_ADAPTER}" -eq 1 ]]; then
    EXPECTED_WAYPOINTS=5
    [[ "${WP_MODEL}" != "nomad" && "${WP_MODEL}" != "omnivla_edge" ]] || EXPECTED_WAYPOINTS=8
    "${SCRIPT_DIR}/run_gnm_path_adapter_whill.sh" --expected-waypoints "${EXPECTED_WAYPOINTS}" \
        --source-frame "${PATH_SOURCE_FRAME}" --target-frame "${PATH_TARGET_FRAME}" &
    PATH_ADAPTER_PID=$!
fi

cleanup()
{
    [[ -z "${VIZ_PID}" ]] || kill "${VIZ_PID}" 2>/dev/null || true
    [[ -z "${PATH_ADAPTER_PID}" ]] || kill "${PATH_ADAPTER_PID}" 2>/dev/null || true
    [[ -z "${NAV_PID}" ]] || kill "${NAV_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "${SHOW_VISUALIZATION}" -eq 1 ]]; then
    VISUALIZATION_READY=0
    while kill -0 "${NAV_PID}" 2>/dev/null; do
        if [[ -f "${DB_READY_PATH}" ]] && rosnode ping -c 1 /PlaceNavNode >/dev/null 2>&1; then
            VISUALIZATION_READY=1
            break
        fi
        sleep 1
    done
    if [[ "${VISUALIZATION_READY}" -eq 1 ]] && kill -0 "${NAV_PID}" 2>/dev/null; then
        echo "Starting PlaceNav GUI on DISPLAY=${DISPLAY:-<unset>}"
        "${SCRIPT_DIR}/visualize_whill.sh" \
            --topomap-dir "${TOPOMAP_DIR}" \
            --pr-model "${PR_MODEL}" \
            --show &
        VIZ_PID=$!
        sleep 1
        if ! kill -0 "${VIZ_PID}" 2>/dev/null; then
            echo "PlaceNav GUI exited during startup. Check the error above and the container DISPLAY/X11 settings." >&2
        fi
    else
        echo "Visualization was not started: navigation exited or ${DB_READY_PATH} is missing." >&2
    fi
fi

set +e
# Remote navigation: the adapter is the local process whose exit ends the session.
wait "${NAV_PID:-${PATH_ADAPTER_PID}}"
NAV_STATUS=$?
set -e
exit "${NAV_STATUS}"
