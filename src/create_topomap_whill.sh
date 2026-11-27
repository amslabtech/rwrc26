#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG="${SCRIPT_DIR}/config/robots.yaml"
TOPOMAP_DIR="${SCRIPT_DIR}/topomaps/images"

sampling_mode=adaptive
straight_distance=2.0
yaw_threshold=0.2618

ROUTE_NAME="dkanaround_${sampling_mode}_${straight_distance}_${yaw_threshold}"

SAMPLE_PERIOD=3.0

exec python3 "${SCRIPT_DIR}/placenav/create_topomap.py" \
    --robot whill \
    --robot_config_path "${ROBOT_CONFIG}" \
    --route_name "${ROUTE_NAME}" \
    --dt "${SAMPLE_PERIOD}" \
    --sampling-mode ${sampling_mode} \
    --straight-distance ${straight_distance} \
    --turn-distance 0.15 \
    --yaw-threshold ${yaw_threshold} \
    --min-distance 0.10 \
    --topomap_directory "${TOPOMAP_DIR}" \
    --odom-topic /odom
