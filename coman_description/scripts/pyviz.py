#!/usr/bin/env python
""" Run Agnathax visualisation """

import time
import rospy
import roslaunch

from gen_urdf import generate_robot
from load_default_parameters import rosparam_load_default_parameters


def main(namespace="/agnathax/"):
    """ Main """
    # Load robot parameters
    rosparam_load_default_parameters(namespace+"parameters")
    # Force time to not come from simulation (otherwise joints not updated)
    rospy.set_param("/use_sim_time", False)
    # Load robot
    rospy.set_param(
        namespace+"robot_description",
        generate_robot().to_xml_string()
    )
    node_rviz = roslaunch.core.Node(
        package="rviz",
        node_type="rviz",
        name="rviz",
        namespace=namespace,
        respawn=True
    )
    rospy.set_param(namespace+"use_gui", True)
    node_joints = roslaunch.core.Node(
        package="joint_state_publisher",
        node_type="joint_state_publisher",
        name="joint_state_publisher",
        namespace=namespace
    )
    node_state = roslaunch.core.Node(
        package="robot_state_publisher",
        node_type="state_publisher",
        name="robot_state_publisher",
        namespace=namespace
    )
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    processes = [
        launch.launch(node)
        for node in [node_rviz, node_joints, node_state]
    ]
    # Loop
    while any([process.is_alive() for process in processes]):
        time.sleep(2)
    # Close
    for process in processes:
        process.stop()
    return


if __name__ == '__main__':
    main()
