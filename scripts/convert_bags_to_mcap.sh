#!/usr/bin/env bash
# Convert ROS 2 sqlite3 bag directories to mcap format for Foxglove.
# Usage:
#   ./scripts/convert_bags_to_mcap.sh
#
# Requires: workspace sourced (e.g. source install/setup.bash) so message types are found.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAGS_DIR="${1:-${REPO_ROOT}/bags/real/zenxin}"  # NOTE:Change here
STORAGE_CONFIG="${REPO_ROOT}/src/uosm_uav_bringup/config/mcap_writer_options.yaml"

if [[ ! -d "$BAGS_DIR" ]]; then
  echo "Bags directory not found: $BAGS_DIR"
  exit 1
fi

# Decompress .db3.zst in a bag dir and fix metadata so ros2 bag can read it.
decompress_zst_bag() {
  local dir="$1"
  local need_fix=0
  for zst in "$dir"/*.db3.zst; do
    [[ -f "$zst" ]] || continue
    base="${zst%.zst}"  # e.g. test1_0.db3
    if [[ ! -f "$base" ]]; then
      echo "  Decompressing $(basename "$zst") -> $(basename "$base")"
      zstd -d -f -q "$zst" || { echo "zstd failed. Install with: sudo apt install zstd"; return 1; }
      need_fix=1
    fi
  done
  local meta="${dir}metadata.yaml"
  if [[ -f "$meta" ]]; then
    if [[ "$need_fix" -eq 1 ]]; then
      # Fix relative_file_paths: .db3.zst or .db3.zst.zst -> .db3
      sed -i -E 's|\.db3\.zst(\.zst)*|.db3|g' "$meta"
    fi
    # If metadata says zstd but relative_file_paths already reference plain .db3
    # files (either freshly decompressed or a previously partial fix), rosbag2
    # would try to decompress again and fail. Clear compression unconditionally.
    if grep -q 'compression_format: zstd' "$meta" 2>/dev/null && \
       ! grep -q '\.db3\.zst' "$meta" 2>/dev/null && \
       grep -q '\.db3' "$meta" 2>/dev/null; then
      sed -i -E 's/(compression_format:).*/\1 ""/' "$meta"
      sed -i -E 's/(compression_mode:).*/\1 ""/' "$meta"
    fi
  fi
}

cd "$REPO_ROOT"
for bag_dir in "$BAGS_DIR"/*/; do
  [[ -d "$bag_dir" ]] || continue
  name="$(basename "$bag_dir")"
  if [[ -f "${bag_dir}metadata.yaml" ]]; then
    out_uri="${bag_dir%/}_mcap"
    if [[ -d "$out_uri" ]]; then
      echo "Skipping $name (output exists: $out_uri)"
      continue
    fi
    echo "Converting $name -> ${name}_mcap"
    decompress_zst_bag "$bag_dir" || exit 1
    # ros2 bag convert requires a YAML; generate it inline (all topics -> mcap)
    out_uri_abs="$(realpath -m "${out_uri}")"
    conv_file="${SCRIPT_DIR}/.convert_to_mcap_oneoff.yaml"
    if [[ -f "$STORAGE_CONFIG" ]]; then
      storage_uri="$(realpath "${STORAGE_CONFIG}")"
      cat > "$conv_file" << EOF
output_bags:
  - uri: ${out_uri_abs}
    storage_id: mcap
    storage_config_uri: ${storage_uri}
    all: true
EOF
    else
      cat > "$conv_file" << EOF
output_bags:
  - uri: ${out_uri_abs}
    storage_id: mcap
    all: true
EOF
    fi
    if ! ros2 bag convert -i "$(realpath "$bag_dir")" -o "$conv_file"; then
      echo "Conversion failed for $name"
      rm -f "$conv_file"
      exit 1
    fi
    rm -f "$conv_file"
  fi
done
echo "Done. MCAP bags are in *_mcap/ next to each original bag. Open the .mcap file inside each folder in Foxglove."
