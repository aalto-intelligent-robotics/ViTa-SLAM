#!/bin/bash

# default is to launch whiskeye world
WORLD=$1
if [ "$WORLD" == "" ]; then WORLD=whiskeye; fi

# local paths
DIR_ROOT=`pwd | sed 's/WhiskEye.*/WhiskEye/'`

# set paths
export GAZEBO_MODEL_PATH=${DIR_ROOT}:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${DIR_ROOT}/whiskeye_robot:${GAZEBO_PLUGIN_PATH}
#export GAZEBO_RESOURCE_PATH=.:${GAZEBO_RESOURCE_PATH}
env | grep GAZ

# launch options
VERBOSE=--verbose
#PAUSE=--pause

# launch
gazebo $WORLD.world $VERBOSE $PAUSE

