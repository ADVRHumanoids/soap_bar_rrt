#!/usr/bin/env python
import xbot_interface.config_options as xbot_opt
import xbot_interface.xbot_interface as xbot
import rospy
from multi_contact_planning.srv import StiffnessDamping
import numpy as np
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

init_stiff = dict()

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
    print robot.arm(0).getJointNames()
    K1 = robot.arm(1).getStiffness()
    print robot.arm(1).getJointNames()
    K2 = robot.leg(0).getStiffness()
    print robot.leg(0).getJointNames()
    K3 = robot.leg(1).getStiffness()
    print robot.leg(1).getJointNames()


    Kd0 = list()
    Kd1 = list()
    Kd2 = list()
    Kd3 = list()

    scale = 1.5

    if chain == "left_arm":
        print ['case: left_arm .... chain: ', chain]
        Kd0 = [1400 * scale, 1000 * scale, 1000 * scale, 1000 * scale, 1000 * scale, 1000 * scale, 1000 * scale]
        Kd1 = K1
        Kd2 = K2
        Kd3 = K3
    elif chain == "right_arm":
        print ['case: right_arm .... chain: ', chain]
        Kd0 = K0
        Kd1 = [1400 * scale, 1000 * scale, 1000 * scale, 1000 * scale, 1000 * scale, 1000 * scale, 1000 * scale]
        Kd2 = K2
        Kd3 = K3
    elif chain == "left_leg":
        print ['case: left_leg .... chain: ', chain]
        Kd0 = K0
        Kd1 = K1
        Kd2 = [800 * scale, 800 * scale, 800 * scale, 800 * scale, 800 * scale, 800 * scale]
        Kd3 = K3
    elif chain == "right_leg":
        print ['case: right_leg .... chain: ', chain]
        Kd0 = K0
        Kd1 = K1
        Kd2 = K2
        Kd3 = [800 * scale, 800 * scale, 800 * scale, 800 * scale, 800 * scale, 800 * scale]



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
        rospy.sleep(1.0/N_ITER)

    print "Stiffness of robot is: ", robot.getStiffness()

def executeCommand(req) :

    # .... do interpolation #
    setStiffnessAndDamping(robot, req.n_iter, req.multiplier, req.chain)
    return True

def resetCommand(req):
    resetStiffness(robot, req.n_iter)
    return True

def resetStiffness(robot, N_ITER):
    global init_stiff
    K = robot.getStiffnessMap()
    Kd = init_stiff
    for k in range(N_ITER):
        stiff = dict()
        for name, K_s, K_e in zip(K, K.values(), Kd.values()):
            stiff[name] = (K_s + float(k) / (N_ITER - 1) * (K_e - K_s))
        robot.setStiffness(stiff)
        robot.move()
        rospy.sleep(1.0/N_ITER)

    print "Stiffness of robot is: ", robot.getStiffness()

def initStiffness(EmptyRequest):
    setInitStiff(robot)
    print 'init stiffness: '
    print robot.getStiffness()
    return EmptyResponse()

def setInitStiff(robot):
    stiff = dict()
    scale = 2.
    stiff = robot.getStiffnessMap()
    stiff['RShSag'] = 700 * scale
    stiff['RShLat'] = 500 * scale
    stiff['RShYaw'] = 500 * scale
    stiff['RElbj'] = 500 * scale
    stiff['RForearmPlate'] = 500 * scale
    stiff['RWrj1'] = 500 * scale
    stiff['RWrj2'] = 500 * scale
    stiff['LShSag'] = 700 * scale
    stiff['LShLat'] = 500 * scale
    stiff['LShYaw'] = 500 * scale
    stiff['LElbj'] = 500 * scale
    stiff['LForearmPlate'] = 500 * scale
    stiff['LWrj1'] = 500 * scale
    stiff['LWrj2'] = 500 * scale
    stiff['WaistYaw'] = 1000 * scale
    stiff['WaistLat'] = 400 * scale
    stiff['LHipLat'] = 400 * scale
    stiff['LHipSag'] = 400 * scale
    stiff['LHipYaw'] = 400 * scale
    stiff['LKneePitch'] = 400 * scale
    stiff['LAnklePitch'] = 400 * scale
    stiff['LAnkleRoll'] = 400 * scale
    stiff['RHipLat'] = 400 * scale
    stiff['RHipSag'] = 400 * scale
    stiff['RHipYaw'] = 400 * scale
    stiff['RKneePitch'] = 400 * scale
    stiff['RAnklePitch'] = 400 * scale
    stiff['RAnkleRoll'] = 400 * scale

    global init_stiff
    init_stiff = stiff

    K = robot.getStiffnessMap()
    Kd = stiff
    for k in range(100):
        s = dict()
        for name, K_s, K_e in zip(K, K.values(), Kd.values()):
            s[name] = (K_s + float(k) / (100 - 1) * (K_e - K_s))
        robot.setStiffness(s)
        robot.move()
        rospy.sleep(1.0/100)

def ankleStiff(EmptyRequest):
    setAnkleStiff(robot)
    return EmptyResponse()

def setAnkleStiff(robot):
    K = robot.getStiffnessMap()
    Kd = K
    Kd['LAnklePitch'] = 500
    Kd['LAnkleRoll'] = 500
    Kd['LAnklePitch'] = 500
    Kd['LAnkleRoll'] = 500
    for k in range(100):
        s = dict()
        for name, K_s, K_e in zip(K, K.values(), Kd.values()):
            s[name] = (K_s + float(k) / (100 - 1) * (K_e - K_s))
        robot.setStiffness(s)
        robot.move()
        rospy.sleep(1.0/100)

    print "Stiffness of robot is: ", robot.getStiffness()

if __name__ == '__main__':

    rospy.init_node('xbot_mode_handler')
    if roscpp_init('xbot_mode_handler', []):
        print 'Unable to initialize roscpp node!'
    robot, model = get_robot()
    robot.sense()
    model.syncFrom(robot)

    robot.setControlMode(xbot.ControlMode.Stiffness())
    global init_stiff
    init_stiff = robot.getStiffnessMap()

    rospy.Service('xbot_mode/set_stiffness_damping', StiffnessDamping, executeCommand)
    rospy.Service('xbot_mode/reset_stiffness', StiffnessDamping, resetCommand)
    rospy.Service('xbot_mode/init_stiffness', Empty, initStiffness)
    rospy.Service('xbot_mode/ankle_stiffness', Empty, ankleStiff)

    while not rospy.is_shutdown():
        robot.sense()
        rospy.spin()