#!/bin/bash
# patch_acados_vendor.sh
# Post-build patches for acados_vendor_ros2 on Jetson (aarch64)
#
# Fixes:
#   1. Replaces x86_64 t_renderer binary with aarch64 version
#   2. Removes legacy 'future_fstrings' encoding incompatible with Python 3.10+
#
# Usage:
#   ./scripts/patch_acados_vendor.sh [WORKSPACE_DIR]
#
#   WORKSPACE_DIR defaults to the colcon workspace root (4 levels up from this script)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${1:-$(realpath "$SCRIPT_DIR/../../..")}"

ACADOS_INSTALL="$WS_DIR/install/acados_vendor_ros2"
ACADOS_BUILD="$WS_DIR/build/acados_vendor_ros2"
TERA_BIN="$ACADOS_INSTALL/opt/acados_vendor_ros2/bin/t_renderer"
TERA_VERSION="0.2.0"

echo "=== Patching acados_vendor_ros2 ==="
echo "Workspace: $WS_DIR"

# --- Patch 1: t_renderer binary for aarch64 ---
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    # Check if existing binary is already aarch64
    if [ -f "$TERA_BIN" ] && file "$TERA_BIN" | grep -q "ARM aarch64"; then
        echo "[tera] Already aarch64, skipping."
    else
        echo "[tera] Downloading t_renderer v${TERA_VERSION} for aarch64..."
        curl -fsSL -o "$TERA_BIN" \
            "https://github.com/acados/tera_renderer/releases/download/v${TERA_VERSION}/t_renderer-v${TERA_VERSION}-linux-arm64"
        chmod +x "$TERA_BIN"
        echo "[tera] Done."
    fi
else
    echo "[tera] Architecture is $ARCH, no patch needed."
fi

# --- Patch 2: Remove future_fstrings encoding declaration ---
ACADOS_PY_DIR="$ACADOS_BUILD/acados_vendor_ros2-prefix/src/acados_vendor_ros2/interfaces/acados_template/acados_template"
if [ -d "$ACADOS_PY_DIR" ]; then
    COUNT=$(grep -rl 'coding: future_fstrings' "$ACADOS_PY_DIR" 2>/dev/null | wc -l || true)
    if [ "$COUNT" -gt 0 ]; then
        echo "[python] Fixing future_fstrings encoding in $COUNT files..."
        find "$ACADOS_PY_DIR" -type f \( -name '*.py' -o -name '*.pyx' -o -name '*.pxd' \) \
            -exec sed -i 's/# -\*- coding: future_fstrings -\*-/# -*- coding: utf-8 -*-/' {} +
        echo "[python] Done."
    else
        echo "[python] No future_fstrings found, skipping."
    fi
else
    echo "[python] Warning: acados_template source not found at $ACADOS_PY_DIR"
fi

echo "=== Patch complete ==="
