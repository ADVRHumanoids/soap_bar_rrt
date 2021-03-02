#!/usr/bin/env python
import xbot_interface.config_options as xbot_opt
import xbot_interface.xbot_interface as xbot
import rospy
from multi_contact_planning.srv import StiffnessDamping
import numpy as np
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
from std_srvs.srv import Empty

def is_empty(any_structure):
    if any_structure:
        return False
    else:
        return True

def get_robot() :

    np.set_printoptions(precision=3, suppress=True)

    # INIT FOR XBOTCORE
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('robot_description')
    srdf = rospy.get_param('robot_description_semantic')

    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('framework', 'ROS')

    robot = xbot.RobotInterface(opt)
    model = xbot.ModelInterface(opt)

    return robot, model

def setStiffnessAndDamping(robot, N_ITER, multiplier, chain):

    K0 = robot.arm(0).getStiffness()
    K1 = robot.arm(1).getStiffness()
    K2 = robot.leg(0).getStiffness()
    K3 = robot.leg(1).getStiffness()

    Kd0 = list()
    Kd1 = list()
    Kd2 = list()
    Kd3 = list()
    if chain == "left_arm":
        Kd0 = multiplier * K0
        Kd1 = K1
        Kd2 = K2
        Kd3 = K3
    elif chain == "right_arm":
        Kd0 = K0
        Kd1 = multiplier * K1
        Kd2 = K2
        Kd3 = K3
    elif chain == "left_leg":
        Kd0 = K0
        Kd1 = K1
        Kd2 = multiplier * K2
        Kd3 = K3
    elif chain == "right_leg":
        Kd0 = K0
        Kd1 = K1
        Kd2 = K2
        Kd3 = multiplier * K3

    for k in range(N_ITER):
        stiff0 = list()
        stiff1 = list()
        stiff2 = list()
        stiff3 = list()
        for K_s0, K_e0 in zip(K0, Kd0):
            stiff0.append(K_s0 + float(k) / (N_ITER - 1) * (K_e0 - K_s0))
        for K_s1, K_e1 in zip(K1, Kd1):
            stiff1.append(K_s1 + float(k) / (N_ITER - 1) * (K_e1 - K_s1))
        for K_s2, K_e2 in zip(K2, Kd2):
            stiff2.append(K_s2 + float(k) / (N_ITER - 1) * (K_e2 - K_s2))
        for K_s3, K_e3 in zip(K3, Kd3):
            stiff3.append(K_s3 + float(k) / (N_ITER - 1) * (K_e3 - K_s3))
        robot.arm(0).setStiffness(stiff0)
        robot.arm(1).setStiffness(stiff1)
        robot.leg(0).setStiffness(stiff2)
        robot.leg(1).setStiffness(stiff3)
        robot.move()
        rospy.sleep(0.5/N_ITER)

    print "Stiffness of robot is: ", robot.getStiffness()

def executeCommand(req) :

    # .... do interpolation #
    setStiffnessAndDamping(robot, req.n_iter, req.multiplier, req.chain)
    return True

def resetCommand(req):
    resetStiffness(robot, req.n_iter)
    return True

def resetStiffness(robot, N_ITER):
    K = robot.getStiffness()
    Kd = init_stiff
    for k in range(N_ITER):
        stiff = list()
        for K_s, K_e in zip(K, Kd):
            stiff.append(K_s + float(k) / (N_ITER - 1) * (K_e - K_s))
        robot.setStiffness(stiff)
        robot.move()
        rospy.sleep(0.5/N_ITER)

if __name__ == '__main__':

    rospy.init_node('xbot_mode_handler')
    if roscpp_init('xbot_mode_handler', []):
        print 'Unable to initialize roscpp node!'
    robot, model = get_robot()
    robot.sense()
    model.syncFrom(robot)

    robot.setControlMode(xbot.ControlMode.Stiffness())
    init_stiff = robot.getStiffness()

    s = rospy.Service('xbot_mode/set_stiffness_damping', StiffnessDamping, executeCommand)
    r = rospy.Service('xbot_mode/reset_stiffness', StiffnessDamping, resetCommand)
    while not rospy.is_shutdown():
        robot.sense()
        rospy.spin()