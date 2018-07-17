#!/usr/bin/env python
""" Run Agnathax visualisation """

import time
import rospy
import roslaunch
import subprocess
import rospkg
import numpy as np

from spawn import spawn_node
from gen_camera_urdf import generate_robot as cam_generate_robot
from urdf_parser import parse_model, get_revolute_joints

def setupRobots(islandNamespace, robotNamespace, island_id, robot_id):
    """ Setup nodes and params """
    islandNs = islandNamespace + "_{}/".format(island_id)
    island = "island_{}".format(island_id)

    namespace = islandNs + robotNamespace + "_{}/".format(robot_id)
    rospy.set_param(
        "{}_{}_{}_{}".format(islandNamespace, island_id, robotNamespace, robot_id),
        namespace
    )

    rospack = rospkg.RosPack()
    rospack.list()

    root_path = rospack.get_path('coman_description')
    model_path = root_path + "/model/model.urdf"

    robot = parse_model(model_path)

    rospy.set_param(
        namespace+"robot_description",
        robot.to_xml_string()
    )

    joints = get_revolute_joints(robot)

    for joint_i, joint in enumerate(joints):
        # print(joint.name)
        rospy.set_param(
            namespace+"joint/{}".format((joint_i)),
            joint.name
        )
        # print(str(joint_i) + " : " + 
        #     rospy.get_param(
        #         namespace+"joint/{}".format((joint_i))
        #     )
        # )

    cam_robot = cam_generate_robot(namespace)
    rospy.set_param(
        namespace+"camera",
        cam_robot.to_xml_string()
    )
    
    node_state = roslaunch.core.Node(
        package="robot_state_publisher",
        node_type="state_publisher",
        name="robot_state_publisher",
        namespace=namespace
    )

    spawnNodes = spawn_node(islandNamespace=islandNs ,namespace=namespace, island_id=island_id, robot_id=robot_id, model_location=[0, 0, 0], model_rotation=[0, 0, 0], camera_location=[1, 1, 0.2])
    
    # robot node
    spawnRobotNode = spawnNodes[0]
    # camera node
    spawnCameraNode = spawnNodes[1]

    nodes = [
        spawnRobotNode,
        spawnCameraNode,
        node_state
    ]
    return nodes

def setupIslands(islandNamespace, island_id, robots):
    """ Setup nodes and params """
    nodes = []
    # Load model pramaters
    namespace = islandNamespace + "_{}/".format(island_id)

    node_gzserver = roslaunch.core.Node(
        package="gazebo_ros",
        node_type="gzserver",
        name="gzserver",
        respawn=False,
        namespace=namespace
    )
    node_gzclient = roslaunch.core.Node(
        package="gazebo_ros",
        node_type="gzclient",
        name="gzclient",
        namespace=namespace,
        respawn=True
    )
    arg_gazebo = "call --wait {}/pause_physics".format(namespace+"gzserver")
    print(arg_gazebo)
    node_gzstate = roslaunch.core.Node(
        # rosservice call gazebo/pause_physics
        package="rosservice",
        node_type="rosservice",
        name="gazebo_properties",
        namespace=namespace,
        args=(
            arg_gazebo
        )
    )
    nodes.append(node_gzserver)
    nodes.append(node_gzclient)
    nodes.append(node_gzstate)

    for robot_id in range(1, robots+1):
        nodes.extend(setupRobots(islandNamespace, "robot", island_id, robot_id))
        
    return nodes


def launch_sim(islands, robots):
    """ Launch simulation with Gazebo """
    nodes=[]
    processes=[]
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    for island_id in range(1, islands+1):
        robot = robots[island_id-1]
        nodes.extend(setupIslands("/island", island_id, robot))

    processes = [launch.launch(node) for node in nodes]

    # Loop
    while any([process.is_alive() for process in processes]):
        time.sleep(2)
    # Close
    for process in processes:
        process.stop()
    return


def main():
    """ Main """
    islands = 1
    robots = [1]

    roscore = subprocess.Popen("roscore -p 11311", shell=True)
    
    # @param1 number of islands
    # @param2 number of robots per island 
    # @example: launch_sim(2, [1, 2]) : island 1 has 1 model and island 2 has 2 models
    # time.sleep(1)
    launch_sim(islands, robots)

    roscore.kill()
    return


if __name__ == '__main__':
    main()
