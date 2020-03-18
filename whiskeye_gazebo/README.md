# WhiskEye_Gazebo

![WhiskEye Robot in Gazebo](whiskeye_square_world.jpg)

## Description
WhiskEye related files like launch files, world files and plugins

| Filename 					   | Description |
|------------------------------|-------------------------------------------------|
|  `whiskeye_ball_world.launch`|Launch file to setup Whiskeye, arena with 3 balls|
|  `whiskeye_empty.launch`     |Launch file to setup empty world                 |
|  `whiskeye_circle_world.launch`     |Launch file to setup empty circle world                 |
|  `whiskeye_square_world.launch`     |Launch file to setup square world with 2 static landmarks and the whiskeye robot. The launchfile is modified to use the robot gazebo model with the third camera from the body_camera branch|

# Troubleshooting
- If the elements are not visible in Gazebo, be sure to launch Gazebo in `verbose` mode. 
- Sometimes, the mesh files are not found. This is because Gazebo is looking for them in the default location. For this add the following directives in the bashrc:
- `export GAZEBO_MODEL_PATH=$HOME/catkin_whiskeye/src/whiskeye_description/:${GAZEBO_MODEL_PATH}`
- `export GAZEBO_RESOURCE_PATH=$HOME/catkin_whiskeye/src/whiskeye_gazebo/worlds/:${GAZEBO_RESOURCE_PATH}`
- `export GAZEBO_PLUGIN_PATH=$HOME/catkin_whiskeye/src/whiskeye_description/whiskeye_robot:${GAZEBO_PLUGIN_PATH}`
