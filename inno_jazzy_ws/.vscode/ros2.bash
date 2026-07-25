if [ -f "$HOME/.bashrc" ]; then
  source "$HOME/.bashrc"
fi

source /opt/ros/jazzy/setup.bash

if [ -f "/home/seeno04/inno_jazzy_ws/install/setup.bash" ]; then
  source "/home/seeno04/inno_jazzy_ws/install/setup.bash"
fi
