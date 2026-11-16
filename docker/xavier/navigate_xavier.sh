#!/usr/bin/env bash
# [Xavier] Run navigate.py for the navigation.launch that parrot started with
# remote_navigation:=true. Route, models and indices are read from parrot's
# /placenav_runner parameters, so nothing is repeated here. Stops when that launch ends.
#
#   parrot:  roslaunch rwrc26_bringup navigation.launch remote_navigation:=true path_timeout:=1.0 \
#                wp_model:=omnivla_edge pr_model:=cosplace topomap_dir:=<route> ...
#   Xavier:  /mnt/ssd/PlaceNav/docker/xavier/navigate_xavier.sh
#
# Before starting, the Xavier clock must follow parrot (chrony): stamps of camera frames
# (parrot) and of waypoints (here) are compared by TF and by path_timeout.
set -euo pipefail
source "$(dirname "$0")/common.sh"
MAX_CLOCK_OFFSET=${MAX_CLOCK_OFFSET:-0.005}   # s

if [[ "${1:-}" != "--skip-clock-check" ]]; then
    command -v chronyc >/dev/null || die "chronyc not found; is chrony installed? (--skip-clock-check to ignore)"
    tracking=$(chronyc tracking) || die "chronyc tracking failed"
    reference=$(awk -F'[()]' '/^Reference ID/ {print $2}' <<<"${tracking}")
    offset=$(awk '/^System time/ {print $4}' <<<"${tracking}")
    [[ "${reference}" == "${PARROT_IP}" ]] \
        || die "chrony follows '${reference}', not parrot ${PARROT_IP}. Check /etc/chrony/chrony.conf (--skip-clock-check to ignore)"
    awk -v o="${offset}" -v m="${MAX_CLOCK_OFFSET}" 'BEGIN {exit !(o ~ /^[0-9]+(\.[0-9]+)?$/ && o + 0 <= m + 0)}' \
        || die "clock is ${offset} s off parrot (> ${MAX_CLOCK_OFFSET} s); chrony is still converging (e.g. after a reboot). Retry in a minute."
    echo "clock: following parrot, ${offset} s off"
fi

exec "$(dirname "$0")/run_xavier.sh" python3 rwrc26_bringup/scripts/session_runner.py --xavier
