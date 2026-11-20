#!/usr/bin/env bash
set -e
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/humble/setup.bash
source /opt/isaac_ros_ws/install/setup.bash
export ROS_DOMAIN_ID=26 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$here/gpu_peer.xml"
# Only one live launch should own these topics in this container.
exec 9>"/tmp/astribot-foundationpose-$ROS_DOMAIN_ID.lock"
flock -n 9 || { echo "Astribot FoundationPose is already running." >&2; exit 1; }
echo "$$" >"/tmp/astribot-foundationpose-$ROS_DOMAIN_ID.pid"
exec ros2 launch "$here/astribot.launch.py" "$@"
