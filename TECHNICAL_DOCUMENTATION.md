# Technical Report
TODO more tech stuff

## Installation
More installation instructions can be found in src/master\_node/README.md

## Usage
If you want to use only the ViTa-SLAM package you can include the pacakge in your catkin\_workspace and use the included launchfile.
ViTa-SLAM consists of 3 nodes: the pose cell node, the experience map node and visuo-tactile template matching node.

An example of how ViTa-SLAM is used in our experiment for the publication [1] can be run as follows:
There are two main launchfiles to start ViTa-SLAM:
- bagfile.launch: This launchfile starts ViTa-SLAM with a bagfile as input. 
The bagfiles can be downloaded from [here](https://aaltofi-my.sharepoint.com/:f:/g/personal/oliver_struckmeier_aalto_fi/Eo5QWmnFoqpKpH-5Rs5BLEsBHkuCW81LPBeS68JeMQNp4A?e=tTxh2g).

TODO
- master.launch: This launchfile starts ViTa-SLAM with Gazebo. A gazebo world is specified in the src/whiskeye\_gazebo package and a variety of controllers to control the whiskeye robot platform are started. For more details see the comments in the launchfile.

## NRP
In order for the ViTa-SLAM algorithm to work with the NRP, the vitaslam package should be moved to the gazebo workspace at `$NRP/GazeboRosPackages/src`.
Then the workspace needs to be compiled with `catkin_make`.
Since the NRP uses personalized Gazebo packages, in $NRP/GazeboRosPackages/src/gazebo\_ros\_pkgs/gazebo\_ros/launch, the argument `<arg name="recording" default="false"/>` should be added inside empty\_world.launch.
ViTa-SLAM can be launched with a NRP experiment by adding the "rosLaunch" parameter in the experiment configuration file with the location of the launch file of ViTa-SLAM.
ViTa-SLAM then acts as just another ROS node and can be used according to the specifications in this document.
Since the topic of the odometry sensor of the WhiskEye robot is not published, in the Transfer Function of the NRP experiment the odometry data must be published in the repsective topic manually.

## Parameter tuning
The parameters for ViTa-SLAM can be changed in src/vitaslam/config/config\_whiskeye.txt.in.
There are many parameters that influence the performance of ViTa-SLAM.
The meaning and influence of the most important parameters is explained in the tuning\_parameters\guide.txt.

## The WhiskEye robot
The sources (models and plugins) for the WhiskEye robot platform are contained on a separate branch "whiskeye_robot" in this repository.
There is also mroe information about how to use that package.
The WhiskEye robot is provided by Martin J. Pearson from the Bristol Robotics Lab (BRL). (Mail: martin.pearson@brl.ac.uk)
