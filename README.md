# ViTa-SLAM
This is the user documentation of ViTa-SLAM and the contents of this repository.
A more detailed technical report can be found in the `TECHNICAL_REPORT.md`.

## Introduction
ViTa-SLAM[1] is a rat hippocampus-inspired visual SLAM framework capable of generating semi-metric topological representations of indoor and outdoor environments by taking visual and tactile data into account. It is an extension to the vision only RatSLAM[2] and the 6D Whisker-RatSLAM[3].

ViTa-SLAM not only allows the robot to perform natural interactions with the environment whilst navigating, as is normally seen in nature, but also provides a mechanism to fuse non-unique tactile and unique visual data. Compared to the former works, our approach can handle ambiguous scenes in which one sensor alone is not capable of identifying false-positive loop-closures

## About this repository
The ViTa-SLAM software framework in this repository can be used as a method of performing SLAM with odometry, visual and tactile data as input. The output is a semi-metric topologial map and a pose estimate of the robot.

Inputs:
- /whiskeye/odom: Odometry of the robot
- /whiskeye/head/rcp\_state: RCP state of the whisker array (protracting or retracting whiskers)
- /whiskeye/head/contact\_world: Whisker contact points in world frame.
- /whiskeye/head/xy: xy deflection angles of the whiskers (2 values (1 for x defl., 1 for y defl.) for each whisker)
- /whiskeye/platform/cam0: Camera input

Outputs:
- /whiskeye/ExperienceMap/Map: The experience map
- /whiskeye/ExperienceMap/RobotPose: The robot pose
- /whiskeye/log/em: The experience map in a different format

## Packages in this repository
 There are 4 packages overall in the master and experiments branches.
- vitaslam: Contains the vitaslam codebase
The following packages in the experiments branch are not needed to run ViTa-SLAM but to recreate the experiments for our paper[1].
- master\_node: Contains the launch scripts and installation instructions
- whiskeye\_controller: Contains various controllers to move the robot in gazebo
- whiskeye\_gazebo: Contains the gazebo files and worlds to test ViTa-SLAM
Move the contents of this repository in a catkin workspace and compile with `catkin_make`. For more details follow the installation instructions in the technical report.

## Sources
[1]
```
@article{struckmeier2019vita,
  title={ViTa-SLAM: A Bio-inspired Visuo-Tactile SLAM for Navigation while Interacting with Aliased Environments},
  author={Struckmeier, Oliver and Tiwari, Kshitij and Salman, Mohammed and Pearson, Martin J and Kyrki, Ville},
  journal={arXiv preprint arXiv:1906.06422},
  year={2019}
}
```
```
[2]
@article{ball2013openratslam,
  title={OpenRatSLAM: an open source brain-based SLAM system},
  author={Ball, David and Heath, Scott and Wiles, Janet and Wyeth, Gordon and Corke, Peter and Milford, Michael},
  journal={Autonomous Robots},
  volume={34},
  number={3},
  pages={149--176},
  year={2013},
  publisher={Springer}
}
```
```
[3]
@inproceedings{salman2018whisker,
  title={Whisker-ratslam applied to 6d object identification and spatial localisation},
  author={Salman, Mohammed and Pearson, Martin J},
  booktitle={Conference on Biomimetic and Biohybrid Systems},
  pages={403--414},
  year={2018},
  organization={Springer}
}
```
