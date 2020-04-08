# Technical Report
This document contains more detailed instruction on how to install and test the ViTa-SLAM code.

## Usage
If you want to use only the ViTa-SLAM package you can include the package in your catkin\_workspace and use the included launchfile.
ViTa-SLAM consists of 3 nodes: the pose cell node, the experience map node and visuo-tactile template matching node.

## Example experiments
An example of how ViTa-SLAM is used in our experiment for the publication [1] can be run as follows:
There are two main launchfiles to start ViTa-SLAM:
- bagfile.launch: This launchfile starts ViTa-SLAM with a bagfile as input. 
The bagfiles can be downloaded from [here](https://aaltofi-my.sharepoint.com/:f:/g/personal/oliver_struckmeier_aalto_fi/Eo5QWmnFoqpKpH-5Rs5BLEsBHkuCW81LPBeS68JeMQNp4A?e=tTxh2g).
- master.launch: This launchfile starts ViTa-SLAM with Gazebo. A gazebo world is specified in the src/whiskeye\_gazebo package and a variety of controllers to control the whiskeye robot platform are started. For more details see the comments in the launchfile.
This requires that all 4 packages included in the master branch and experiment branch are in your workspace and are compiled (See the installation instructions for more details)

## Installation
These are the instructions on how to set up ViTa-SLAM with the included experiments.
1. Install ROS

   Tested with Ubuntu 18.04 and ROS melodic. (For more details visit http://wiki.ros.org/melodic/Installation/Ubuntu)  

   ```
   # sudo apt update && sudo apt upgrade  
   # sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
   # sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654  
   # sudo apt update  
   # sudo apt install ros-melodic-desktop-full  
   ```

   Set up rosdep:  
   ```
   # sudo rosdep init  
   # rosdep update  
   ```

   Set up the environment:  
   ```
   # echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
   # source ~/.bashrc
   ```

   Set up the workspace:
   Create folder:
   ```
   mkdir -p [dir of your choice]/whiskeye\_ws/src
   cd whiskeye_ws
   catkin_make
   ```

2. Download the code base 
   Check out the vitaslam, master\_node, whiskeye\_controller and whiskeye\_gazebo packages from the master and experiment branches in the src of your catkin workspace.
   Sanity check: Make sure the following packages are in `catkin_ws/src/`:
   - master\_node
   - whiskeye\_gazebo
   - vitaslam
   - whiskeye\_controller

3. Run catkin\_make in whiskeye\_ws/

4. Fix possible compile errors ....

   4.1 Irrlicht
   Irrlicht is a 3D rendering library used to render the posecell network and the experience map.
   It needs to be installed using
   `sudo apt install libirrlicht-dev`
   Otherwise you will get errors like: "undefined reference to `irr::video::IdentityMaterial'"

   4.2 Permission denied
   ROS is not able to create log files and access other ros related files:
   `rosdep fix-permissions`

5. Testing
   Adjust the config file in whiskeye_ws/src/vita-slam/config_whiskeye.txt.in:
   - Update media path depending on your folder structure

   5.1 Testing vitaslam with our rosbags
   Download the rosbags from:
   - https://aaltofi-my.sharepoint.com/:f:/g/personal/oliver_struckmeier_aalto_fi/Eo5QWmnFoqpKpH-5Rs5BLEsBHkuCW81LPBeS68JeMQNp4A?e=tTxh2g

   Start the playback of any rosbag (for testing) with:
   `roslaunch master\_node bagfile.launch bagfile\_path:="path/to/your/bagfile.bag"`

   You should see the experience map, posecell network and view cells window.

   5.2 Testing with gazebo
   To run vitaslam in combination with gazebo instead of the use:
   `roslaunch master_node master.launch`
   You should see the same windows plus gazebo opening.

## NRP
In order for the ViTa-SLAM algorithm to work with the NRP, the vitaslam package should be moved to the gazebo workspace at `$NRP/GazeboRosPackages/src`.
Then the workspace needs to be compiled with `catkin\_make`.

Since the NRP uses personalized Gazebo packages, in $NRP/GazeboRosPackages/src/gazebo\_ros\_pkgs/gazebo\_ros/launch, the argument `<arg name="recording" default="false"/>` should be added inside empty\_world.launch.
ViTa-SLAM can be launched with a NRP experiment by adding the "rosLaunch" parameter in the experiment configuration file with the location of the launch file of ViTa-SLAM.

ViTa-SLAM then acts as just another ROS node and can be used according to the specifications in this document.
Since the topic of the odometry sensor of the WhiskEye robot is not published, in the Transfer Function of the NRP experiment the odometry data must be published in the repsective topic manually.
To set up the Whiskeye robot with the NRP, follow these steps:
1. Move the plugin libwhiskeye_gazebo.so from the folder "whiskeye_robot" to $HBP/GazeboRosPackages/devel/lib
2. Copy the folder whiskeye_msgs in $HBP/GazeboRosPackages/devel/lib/python2.7/dist-packages
3. Move the folders whiskeye_robot and whiskeye_msgs to $HBP/Models
4. Create the symlinks for the models by executing the script "create-symlinks.sh" inside $HBP/Models. Then, in order to create a NRP experiment with the Whiskeye, follow these steps:
1. Create a new empty world environment
2. Copy the sdf model of the robot inside the experiment folder in nrpStorage
3. Add the following line to bibi_configuration.bibi in <ns1:bibi>:
  <ns1:bodyModel robotId="whiskeye_robot">whiskeye_robot.sdf</ns1:bodyModel>
4. Add the following line to experiment_configuration.exc ins <environmentModel>:
  <robotPose robotId="whiskeye_robot" x="0.0" y="0.0" z="0.0" roll="0.0" pitch="0.0" yaw="0.0" />
5. Start the experiment
   
![](media/nrp.gif)

## Parameter tuning
The parameters for ViTa-SLAM can be changed in vitaslam/config/config\_whiskeye.txt.in.
There are many parameters that influence the performance of ViTa-SLAM.
The meaning and influence of the most important parameters is explained in the tuning\_parameters\guide.txt.

## The WhiskEye robot
The sources (models and plugins) for the WhiskEye robot platform are contained on a separate branch "whiskeye_robot" in this repository.
There is also mroe information about how to use that package.
The WhiskEye robot is provided by Martin J. Pearson from the Bristol Robotics Lab (BRL). (Mail: martin.pearson@brl.ac.uk)

## Related Publications
If you use ViTa-SLAM for academic research, please cite it as:
[1]
```
@article{struckmeier2019vita,
  title={ViTa-SLAM: A Bio-inspired Visuo-Tactile SLAM for Navigation while Interacting with Aliased Environments},
  author={Struckmeier, Oliver and Tiwari, Kshitij and Salman, Mohammed and Pearson, Martin J and Kyrki, Ville},
  journal={arXiv preprint arXiv:1906.06422},
  year={2019}
}
```

