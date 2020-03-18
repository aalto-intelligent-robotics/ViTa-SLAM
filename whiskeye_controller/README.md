# WhiskEye Controller Documentation

This project contains the documentaiton for the two trajectory generators that are discussed in detail below.

## Description
This repository contains the following controllers:

| Name                            | Comments                |Execution
|---------------------------------|-------------------------|-------------------------|
|`teleop.py`|This is a keyboard teleop node| `roslaunch whiskeye_controller teleop.launch`|
|`random_walk_circle.py`|Random walking in a circular arena|`roslaunch whiskeye_controller random_walk_circle.launch` |
|`random_walk_square.py`|Random walking in a square arena|`roslaunch whiskeye_controller random_walk_square.launch` |

## Expected Behavior
The figure below shows the expected random walking behavior.
![](random_walk.png)

- Left: Circle Arena without any landmarks. The incidence angle $(\angle i)$ is manipulated by some random angle $(\angle \delta)$ and used as angle of reflectance $(\angle r)$. 
$\angle r = \angle i + \angle \delta$
- Right: Square arena with 2 static landmarks. The incidence angle $(\angle i)$ is used as is for angle of reflectance $(\angle r)$.
$\angle r = \angle i$


## Usage Instructions:
1. Teleop Node:
This node allows manual control over the robot using the following key mapping:
Reading from the keyboard  and Publishing to Twist!

Moving around:

| u | i | o |
|:---:|:---:|:---:|
| j | k | l |
| m | , | . |

- i: forwards
- ,: backwards
- j: rotate left
- l: rotate right
- u: combines i and j
- o: combines i and l
- m: combines j and ,
- .: combines l and ,
  
For Holonomic mode (strafing), hold down the shift key:

| U | I | O |
|:---:|:---:|:---:|
| J | K | L |
| M | < | > |
  
- t : up (+z)
- b : down (-z)
- anything else : stop
- q/z : increase/decrease max speeds by 10%
- w/x : increase/decrease only linear speed by 10%
- e/c : increase/decrease only angular speed by 10%

2. Random Walk Node:
Fully automated with heuristics, see included figures.

3. Teleop neck control
This node allows manual control over the robot neck usig the following key mappings:
Reading from the keyboard and publishing to /whiskeye/head/neck_cmd
- Decrease neck elevation: j (Whole Arm)
- Increase neck eleveation: k  (Whole Arm)
- Decrese neck pitch: u (Only Head)
- Increase neck pitch: i (Only Head)
- Decrease neck yaw: h (Turn Head sideways)
- Increase neck yaw: l (Turn Head sideways)
- Reset to original: r         
- Increase speed: +            
- Decrease speed: -  

4. Teleop whisker control
This node allows manual control over the robot whiskers by setting the angle theta. All 24 (6x4) Whiskers are moved at once. Key mappings:
- Decrease theta: d
- Increase theta: f
- Increase whisker speed: a
- Decrease whisker speed: s

