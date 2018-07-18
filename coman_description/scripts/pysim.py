#!/usr/bin/env python
""" Run Agnathax visualisation """

import time
import rospy
import rospkg
import roslaunch
import subprocess

from spawn import spawn_node
from gen_camera_urdf import generate_robot as cam_generate_robot
from urdf_parser import parse_model, get_revolute_joints, get_all_revolute_joints_names

def setupRobots(islandNamespace, robotNamespace, island_id, robot_id):
    """ Setup nodes and params """
    islandNs = islandNamespace + "_{}/".format(island_id)
    island = "island_{}".format(island_id)
    namespace = islandNs + robotNamespace + "_{}/".format(robot_id)
    
    rospy.set_param(
        "namespace_{}_{}".format(island_id, robot_id),
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

    n_joints = 0
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
        n_joints += 1
    
    print(n_joints)
    arg = namespace+"n_joints"

    rospy.set_param(
        arg,
        n_joints
    )

    cam_robot = cam_generate_robot(namespace)
    rospy.set_param(
        namespace+"camera_description",
        cam_robot.to_xml_string()
    )
    rospy.set_param(
        namespace+"parameters/control/joints",
        get_all_revolute_joints_names(robot)
    )
    node_state = roslaunch.core.Node(
        package="robot_state_publisher",
        node_type="state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        machine_name=island
    )
    spawnNodes = spawn_node(
        islandNamespace=islandNs,
        namespace=namespace, 
        island_id=island_id, 
        robot_id=robot_id, 
        model_location=[
            0, 
            robot_id - 1,
            0
        ], 
        camera_location=[1.2, (robot_id), 0.2]
    )
    # robot node
    spawnRobotNode = spawnNodes[0]
    # camera node
    # spawnCameraNode = spawnNodes[1]

    nodes = [
        spawnRobotNode,
        # spawnCameraNode,
        node_state
    ]
    return nodes

def setupIslands(islandNamespace, island_id, robots):
    """ Setup nodes and params """
    nodes = []
    # Load model pramaters
    namespace = islandNamespace + "_{}/".format(island_id)

    island = "{}_{}".format("island", island_id)
    
    gazebo_env_args = "http://localhost:1134{}".format(4+island_id)
    ros_gazebo_env_args = "/{}".format(island)

    node_gzserver = roslaunch.core.Node(
        package="gazebo_ros",
        node_type="gzserver",
        name="gzserver",
        respawn=False,
        namespace=namespace,
        machine_name=island,
        env_args=[
            ("GAZEBO_MASTER_URI", gazebo_env_args),
            ("ROS_NAMESPACE", ros_gazebo_env_args)
        ]
    )
    node_gzclient = roslaunch.core.Node(
        package="gazebo_ros",
        node_type="gzclient",
        name="gzclient",
        namespace=namespace,
        respawn=True,
        machine_name=island,
        env_args=[
            ("GAZEBO_MASTER_URI", gazebo_env_args),
            ("ROS_NAMESPACE", ros_gazebo_env_args)
        ]
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

    rospy.set_param(
        "islands",
        islands
    )

    rospy.set_param(
        "robots",
        robots
    )

    for island_id in range(1, islands+1):
        robot = robots[island_id-1]
        nodes.extend(setupIslands("/island", island_id, robot))

    for node in nodes:
        print(node.to_xml())
    
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

    # @param1 number of islands
    # @param2 number of robots per island 
    # @example: launch_sim(2, [1, 2]) : island 1 has 1 model and island 2 has 2 models

    islands = 1
    robots = [1]
    
    launch_sim(islands, robots)
    return


if __name__ == '__main__':
    main()
