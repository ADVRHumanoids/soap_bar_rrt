#!/usr/bin/env python
import xbot_interface.config_options as xbot_opt
import xbot_interface.xbot_interface as xbot
import rospy
from multi_contact_planning.srv import StiffnessDamping
import numpy as np
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

def is_empty(any_structure):
    if any_structure:
        return False
    else:
        return True

def executeCommand(req) :

    # .... do interpolation #
    # req.end_effector ...
    # req.stiffness ...
    # req.damping ...

    return True

def get_robot() :

    np.set_printoptions(precision=3, suppress=True)

    # INIT FOR XBOTCORE
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('robot_description')
    srdf = rospy.get_param('robot_semantic_description')

    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('framework', 'ROS')

    robot = xbot.RobotInterface(opt)
    model = xbot.ModelInterface(opt)

    return robot, model

if __name__ == '__main__':

    rospy.init_node('xbot_mode_handler')
    if roscpp_init('xbot_mode_handler', []):
        print 'Unable to initialize roscpp node!'
    robot, model = get_robot()
    robot.sense()
    model.syncFrom(robot)

    robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

    s = rospy.Service('xbot_mode/set_stiffness_damping', StiffnessDamping, executeCommand)

    print("Ready to add two ints.")
    rospy.spin()