#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
openvins_root="${OPENVINS_ROOT:-/root/open_vins}"
dataset="${1:?Usage: $0 DATASET_DIR [OUTPUT_DIR] [MAX_FRAMES]}"
output_dir="${2:-$dataset/results}"
max_frames="${3:-0}"

mkdir -p "$output_dir"
trajectory="$output_dir/trajectory_openvins.csv"
cloud="$output_dir/cloud.pcd"

"$script_dir/build/openvins_kaist_rosfree" \
  "$dataset" \
  "$openvins_root/config/kaist/estimator_config.yaml" \
  "$trajectory" \
  "$max_frames"

"$script_dir/build/kaist_stereo_cloud" \
  "$dataset" "$trajectory" "$cloud" 10 4 15 "$max_frames"

echo "Trajectory: $trajectory"
echo "3D cloud:  $cloud"

