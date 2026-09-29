#!/usr/bin/env bash
set -euo pipefail

robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${robot_root}/run_mode11.sh" use_map:=false "$@"
