#!/usr/bin/env bash
set -e

#Auto-source ROS and workspace overlay in every new shell
SHELLRC="$HOME/.bashrc"
grep -qxF "source /opt/ros/humble/setup.bash" "$SHELLRC" || \
  echo "source /opt/ros/humble/setup.bash" >> "$SHELLRC"
grep -qxF "[ -f /workspaces/anduril_ws/install/setup.bash ] && source /workspaces/anduril_ws/install/setup.bash" "$SHELLRC" || \
  echo "[ -f /workspaces/anduril_ws/install/setup.bash ] && source /workspaces/anduril_ws/install/setup.bash" >> "$SHELLRC"

# Determine if sudo is available and needed
SUDO_CMD=""
if command -v sudo >/dev/null 2>&1; then
    SUDO_CMD="sudo"
fi

# Initialize and update rosdep safely
if command -v rosdep >/dev/null 2>&1; then
    echo "Initializing rosdep..."
    $SUDO_CMD rosdep init 2>/dev/null || true
    rosdep update || true
else
    echo "WARNING: rosdep command not found. Skipping initialization."
fi

# Register Python packages with importlib.metadata so entry points resolve correctly
pip3 install --quiet -e /workspaces/anduril_ws/src/anduril_cv

echo "Dev container ready. Build with:  colcon build"