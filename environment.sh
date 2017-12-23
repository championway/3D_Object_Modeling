#!/bin/bash

echo "Activating ROS..."
source /opt/kinetic/indigo/setup.bash
echo "...done."

echo "Setting up PYTHONPATH."
export PYTHONPATH=/home/ubuntu/3D_Object_Modeling/catkin_ws/src:$PYTHONPATH

echo "Setup ROS_HOSTNAME."
export ROS_HOSTNAME=$HOSTNAME.local
export MODELING_ROOT=$HOME/3D_Object_Modeling

echo "Building machines file..."
make -C  $MODELING_ROOT
echo "...done"
echo "Activating development."
source $MODELING_ROOT/catkin_ws/devel/setup.bash
source $MODELING_ROOT/set_ros_master.sh master

# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.

