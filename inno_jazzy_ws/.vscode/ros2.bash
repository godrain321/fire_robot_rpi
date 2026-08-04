if [ -f "$HOME/.bashrc" ]; then
  source "$HOME/.bashrc"
fi

source /opt/ros/jazzy/setup.bash

INNO_WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "${INNO_WS_DIR}/install/setup.bash" ]; then
  source "${INNO_WS_DIR}/install/setup.bash"
fi
