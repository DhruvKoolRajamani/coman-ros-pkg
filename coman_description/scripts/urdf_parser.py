# Load the urdf_parser_py manifest, you use your own package
# name on the condition but in this case, you need to depend on
# urdf_parser_py.
# import roslib; 
#roslib.load_manifest('urdf_parser_py')

###########################
# Set param for number of joints
###########################

import rospy
import rospkg

# Import the module

from urdf_parser_py.urdf import URDF
from urdf_parser_py import urdf

def get_revolute_joints(robot):
    """ Get robot revolute joints """
    return [joint for joint in robot.joints if joint.type == "revolute"]

def get_joints(robot):
    """ Get robot joints """
    joints = get_revolute_joints(robot)
    # print("Joints:\n{}".format([joint.name for joint in joints]))
    return joints

def get_all_joints_names(robot):
    """ Get all joints of the robot """
    joints = {
        typ: [joint.name for joint in robot.joints if joint.type == typ]
        for typ in urdf.Joint.TYPES
    }
    return joints

def get_all_revolute_joints_names(robot):
    """ Get all joints of the robot """
    typ = "revolute"
    joints = {
        typ: [joint.name for joint in robot.joints if joint.type == "revolute"]
    }
    return joints

def get_all_joint_types(robot):
    """ Get all joint types """
    types = {
        typ: [joint.type for joint in robot.joints if joint.type == typ]
        for typ in urdf.Joint.TYPES
    }
    return types

def ros_param_properties(robot, robot_namespace):
    """ Save robot properties """
    properties = robot_properties(robot)
    rospy.set_param(robot_namespace + "properties", properties)
    return

def robot_properties(robot):
    """ Robot properties """
    return {"jointNames": get_all_revolute_joints_names(robot)}

def parse_model(model_path):

    # 1. Parse a string containing the robot description in urdf.
    # Pro: no need to have a roscore running.
    # Cons: n/a
    # Note: it is rare to receive the robot model as a string.
    desc = open(model_path, 'r')

    robot = URDF.from_xml_string(desc.read())

    # - OR -

    # 2. Load the module from a file.
    # Pro: no need to have a roscore running.
    # Cons: using hardcoded file location is not portable.
    #robot = urdf.from_xml_file()

    # - OR -

    # 3. Load the module from the parameter server.
    # Pro: automatic, no arguments are needed, consistent
    #      with other ROS nodes.
    # Cons: need roscore to be running and the parameter to
    #      to be set beforehand (through a roslaunch file for
    #      instance).
    #robot = urdf.from_parameter_server()

    # Print the robot
    # for element in robot.links:
    #     print(element)
    print(get_all_revolute_joints_names(robot))

    return robot

def main():
    rospack = rospkg.RosPack()
    rospack.list()

    root_path = rospack.get_path('coman_description')
    model_path = root_path + "/model/model.urdf"

    robot = parse_model(model_path)

    # print(robot)

if __name__ == "__main__":
    main()
