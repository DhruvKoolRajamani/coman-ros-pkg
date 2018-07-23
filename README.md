# Coman ROS package

## Install ROS

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`

`sudo apt-get update`

`sudo apt-get install ros-kinetic-desktop`

`sudo rosdep init`
`rosdep update`

`echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc`
`source ~/.bashrc`

`sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential`

## Install Gazebo

``sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'``

`wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -`

`sudo apt-get update`

`sudo apt-get install gazebo7 libgazebo7-dev`

## Install other dependencies

`sudo apt-get install ros-kinetic-gazebo-plugins ros-kinetic-kdl-parser ros-kinetic-kdl-parser-py ros-kinetic-kdl-conversions ros-kinetic-kdl-typekit ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-msgs ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-control-toolbox ros-kinetic-control-msgs ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-joint-limits-interface ros-kinetic-joint-state-controller ros-kinetic-joint-state-publisher ros-kinetic-joint-states-settler ros-kinetic-joint-trajectory-action ros-kinetic-joint-trajectory-action-tools ros-kinetic-joint-trajectory-generator ros-kinetic-srdfdom ros-kinetic-urdf ros-kinetic-urdfdom-py ros-kinetic-urdf-parser-plugin ros-kinetic-gazebo-ros-pkgs`

`sudo apt-get install ros-kinetic-orocos-kdl ros-kinetic-orocos-kinematics-dynamics ros-kinetic-ocl ros-kinetic-rtt ros-kinetic-rtt-actionlib ros-kinetic-rtt-actionlib-msgs ros-kinetic-rtt-common-msgs ros-kinetic-rtt-controller-manager-msgs ros-kinetic-rtt-control-msgs ros-kinetic-rtt-diagnostic-msgs ros-kinetic-rtt-dynamic-reconfigure ros-kinetic-rtt-geometry ros-kinetic-rtt-geometry-msgs ros-kinetic-rtt-kdl-conversions ros-kinetic-rtt-nav-msgs ros-kinetic-rtt-ros ros-kinetic-rtt-rosclock ros-kinetic-rtt-roscomm ros-kinetic-rtt-ros-comm ros-kinetic-rtt-rosdeployment ros-kinetic-rtt-rosgraph-msgs ros-kinetic-rtt-ros-integration ros-kinetic-rtt-ros-msgs ros-kinetic-rtt-rosnode ros-kinetic-rtt-rospack ros-kinetic-rtt-rosparam ros-kinetic-rtt-sensor-msgs ros-kinetic-rtt-shape-msgs ros-kinetic-rtt-std-msgs ros-kinetic-rtt-std-srvs ros-kinetic-rtt-stereo-msgs ros-kinetic-rtt-tf ros-kinetic-rtt-trajectory-msgs ros-kinetic-rtt-visualization-msgs ros-kinetic-eigen-*`

## Instructions

In order to run this package, add it to your catkin workspace. You can use
the following commands:

If you dont have a catkin workspace initialized:

`mkdir -p ~/catkin_ws/src`

`cd catkin_ws/src/`

`git clone git@gitlab.com:DhruvKoolRajamani/coman-ros-pkg.git`

`cd ~/catkin_ws/`

`catkin_make`

Now source your workspace from the catkin base directory and continue: 

`source devel/setup.bash`

Start roscore: `roscore`

`Ctrl+Alt+T`

Gazebo simulaton: `rosrun coman_description pysim.py`

`Ctrl+Alt+T`

Load controllers: `rosrun coman_control controllers.py`

`Ctrl+Alt+T`

Run controller: `rosrun coman_control coman_control_walking`

## Acknowledgement

This branch of the project is under progress by the collaborating team working at the BioRobotics Lab at the Swiss Federal Institute of Technology, Lausanne (Ecole Polytechnic federale de Lausanne).