#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd -- "${script_dir}/.." && pwd)"
repository_dir="$(cd -- "${package_dir}/../../.." && pwd)"
driver_dir="${repository_dir}/mlx90640/demo codes/mlx90640/python/lib"
output_dir="${package_dir}/native"

if [[ ! -f "${driver_dir}/usr_api.cpp" ]]; then
  echo "MLX90640 native source not found: ${driver_dir}" >&2
  exit 1
fi

mkdir -p "${output_dir}"
g++ -std=c++17 -O2 -fPIC -shared \
  -I"${driver_dir}" \
  "${driver_dir}/MLX90640_API.cpp" \
  "${driver_dir}/MLX90640_LINUX_I2C_Driver.cpp" \
  "${package_dir}/native/inno_mlx90640_bridge.cpp" \
  -o "${output_dir}/libmlx90640.so"

echo "Built ${output_dir}/libmlx90640.so"
