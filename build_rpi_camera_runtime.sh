#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_root="${CAMERA_BUILD_ROOT:-${project_root}/.camera_build}"
runtime_root="${CAMERA_RUNTIME_ROOT:-${project_root}/.camera_runtime}"
ros_prefix="${ROS_PREFIX:-/opt/ros/jazzy}"

libcamera_repository='https://github.com/raspberrypi/libcamera.git'
libcamera_tag='v0.7.1+rpt20260609'
libcamera_commit='06c385619acb10bbfb33f52f3abeb8f8c095f42b'
source_dir="${build_root}/libcamera-src"
meson_build_dir="${build_root}/libcamera-build"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

for command_name in git meson ninja pkg-config c++ strings grep realpath readelf ldd sha256sum; do
  command -v "${command_name}" >/dev/null 2>&1 || \
    die "Required command is missing: ${command_name}"
done

(( EUID != 0 )) || \
  die 'Run this script as the normal robot user, not with sudo or as root.'

canonical_project_root="$(realpath -m -- "${project_root}")"
canonical_build_root="$(realpath -m -- "${build_root}")"
canonical_runtime_root="$(realpath -m -- "${runtime_root}")"
expected_build_root="${canonical_project_root}/.camera_build"
expected_runtime_root="${canonical_project_root}/.camera_runtime"
case "${canonical_build_root}" in
  "${expected_build_root}"|/tmp/*) ;;
  *) die "Build root must be ${expected_build_root} or a child of /tmp: ${canonical_build_root}" ;;
esac
[[ "${canonical_runtime_root}" == "${expected_runtime_root}" ]] || \
  die "Runtime root must be exactly ${expected_runtime_root}: ${canonical_runtime_root}"
build_root="${canonical_build_root}"
runtime_root="${canonical_runtime_root}"
source_dir="${build_root}/libcamera-src"
meson_build_dir="${build_root}/libcamera-build"

[[ "$(uname -m)" == 'aarch64' ]] || \
  die 'This runtime is intended for 64-bit Raspberry Pi OS/Ubuntu (aarch64).'
[[ -r "${ros_prefix}/lib/pkgconfig/libpisp.pc" ]] || \
  die "ROS libpisp metadata is missing: ${ros_prefix}/lib/pkgconfig/libpisp.pc"
[[ -r "${ros_prefix}/include/libpisp/backend/backend.hpp" ]] || \
  die "ROS libpisp headers are missing under ${ros_prefix}/include/libpisp."

libpisp_version="$(
  env PKG_CONFIG_PATH="${ros_prefix}/lib/pkgconfig" \
    pkg-config --modversion libpisp
)"
[[ "${libpisp_version}" == '1.3.0' ]] || \
  die "Expected ROS libpisp 1.3.0, found ${libpisp_version}."

mkdir -p -- "${build_root}"

if [[ ! -e "${source_dir}" ]]; then
  printf 'Cloning Raspberry Pi libcamera %s...\n' "${libcamera_tag}"
  git clone --filter=blob:none --depth 1 --branch "${libcamera_tag}" \
    "${libcamera_repository}" "${source_dir}"
elif [[ ! -d "${source_dir}/.git" ]]; then
  die "Build source path exists but is not a Git checkout: ${source_dir}"
fi

actual_remote="$(git -C "${source_dir}" remote get-url origin)"
actual_commit="$(git -C "${source_dir}" rev-parse HEAD)"
[[ "${actual_remote}" == "${libcamera_repository}" ]] || \
  die "Unexpected libcamera remote in ${source_dir}: ${actual_remote}"
[[ "${actual_commit}" == "${libcamera_commit}" ]] || \
  die "Unexpected libcamera commit ${actual_commit}; expected ${libcamera_commit}. Remove ${build_root} and retry."
source_status="$(git -C "${source_dir}" status --porcelain --untracked-files=all)"
[[ -z "${source_status}" ]] || \
  die "Pinned libcamera source is modified. Remove ${build_root} and retry with a clean checkout."

pkg_config_path="${ros_prefix}/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
meson_options=(
  '--wrap-mode=nodownload'
  "--prefix=${runtime_root}"
  '--libdir=lib'
  '--buildtype=release'
  '-Dpipelines=rpi/pisp'
  '-Dipas=rpi/pisp'
  '-Dcam=disabled'
  '-Dqcam=disabled'
  '-Dgstreamer=disabled'
  '-Dpycamera=disabled'
  '-Dv4l2=disabled'
  '-Dtest=false'
  '-Dlc-compliance=disabled'
  '-Ddocumentation=disabled'
)

if [[ -f "${meson_build_dir}/meson-private/coredata.dat" ]]; then
  env PKG_CONFIG_PATH="${pkg_config_path}" \
    meson setup --reconfigure "${meson_build_dir}" "${source_dir}" \
    "${meson_options[@]}"
else
  env PKG_CONFIG_PATH="${pkg_config_path}" \
    meson setup "${meson_build_dir}" "${source_dir}" \
    "${meson_options[@]}"
fi

# Rebuild every generated object on re-runs so a stale/tampered object cannot
# survive merely because its timestamp is newer than the pinned clean source.
ninja -C "${meson_build_dir}" clean

# The ROS libpisp.pc shipped for Jazzy adds .../include/libpisp, while the
# current Raspberry Pi libcamera source includes <libpisp/...>. Add the ROS
# include root explicitly without modifying any system file.
env \
  CPLUS_INCLUDE_PATH="${ros_prefix}/include${CPLUS_INCLUDE_PATH:+:${CPLUS_INCLUDE_PATH}}" \
  C_INCLUDE_PATH="${ros_prefix}/include${C_INCLUDE_PATH:+:${C_INCLUDE_PATH}}" \
  ninja -C "${meson_build_dir}"

meson install -C "${meson_build_dir}"

required_runtime_files=(
  "${runtime_root}/lib/libcamera.so.0.7.1"
  "${runtime_root}/lib/libcamera.so.0.7"
  "${runtime_root}/lib/libcamera-base.so.0.7.1"
  "${runtime_root}/lib/libcamera-base.so.0.7"
  "${runtime_root}/lib/libcamera/ipa/ipa_rpi_pisp.so"
  "${runtime_root}/lib/libcamera/ipa/ipa_rpi_pisp.so.sign"
  "${runtime_root}/share/libcamera/ipa/rpi/pisp/imx708_wide.json"
)
for required_file in "${required_runtime_files[@]}"; do
  [[ -r "${required_file}" ]] || \
    die "Build completed without a required runtime file: ${required_file}"
done

runtime_library="${required_runtime_files[0]}"
strings "${runtime_library}" | grep -F 'rp1-cfe-fe_image0' >/dev/null || \
  die 'Built libcamera does not contain the Pi 5 underscore entity fix.'
readelf -d "${runtime_library}" | \
  grep -F 'Library soname: [libcamera.so.0.7]' >/dev/null || \
  die 'Built libcamera has an unexpected SONAME.'

loader_output="$(
  env LD_LIBRARY_PATH="${runtime_root}/lib:${ros_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    ldd "${runtime_library}"
)"
if grep -F 'not found' <<<"${loader_output}" >/dev/null; then
  printf '%s\n' "${loader_output}" >&2
  die 'Built libcamera has an unresolved runtime dependency.'
fi

manifest="${runtime_root}/BUILD_INFO.txt"
{
  printf 'libcamera_tag=%s\n' "${libcamera_tag}"
  printf 'libcamera_commit=%s\n' "${libcamera_commit}"
  printf 'libpisp_version=%s\n' "${libpisp_version}"
  printf 'meson_version=%s\n' "$(meson --version)"
  printf 'compiler_version=%s\n' "$(c++ -dumpfullversion -dumpversion)"
  sha256sum "${required_runtime_files[@]}"
} >"${manifest}"

printf '\nCamera runtime ready: %s\n' "${runtime_root}"
printf 'Pinned source: %s (%s)\n' "${libcamera_tag}" "${libcamera_commit}"
printf 'Build manifest: %s\n' "${manifest}"
printf 'Next command: ./capture_intrinsic_images.sh\n'
