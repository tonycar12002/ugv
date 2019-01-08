source /opt/ros/kinetic/setup.bash
source $HOME/ugv/catkin_ws/devel/setup.bash

GAZEBO_MODEL_PATH=$HOME/ugv/catkin_ws/src/husky/husky_gazebo/models
export GAZEBO_MODEL_PATH
echo "Set GAZEBO_MODEL_PATH to: $GAZEBO_MODEL_PATH"
