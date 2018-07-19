# Coman ROS package

## Instructions

In order to run this package, add it to your catkin workspace. You can use
the following commands:

If you dont have a catkin workspace initialized: `mkdir -p ~/catkin_ws/src`
Then: `cd catkin_ws/src/`
Clone this repository
Next: `cd ~/catkin_ws/`
Finally: `catkin_make`

Now source your workspace from the catkin base directory and continue: `source devel/setup.bash`
Start roscore: `roscore`

`Ctrl+Alt+T`

Gazebo simulaton: `rosrun coman_description pysim.py`

`Ctrl+Alt+T`

Load controllers: `rosrun coman_control controllers.py`

`Ctrl+Alt+T`

Run controller: `rosrun coman_control coman_control_walking`

## Acknowledgement

CogIMon is a EU-funded research project in the Horizon 2020 Work Programme. The robotics project is part of the Industrial Leadership pillar and hosted by the section for Information and Communication Technologies.

This branch of the project is under progress by the collaborating team working at the BioRobotics Lab at the Swiss Federal Institute of Technology, Lausanne (Ecole Polytechnic federale de Lausanne).