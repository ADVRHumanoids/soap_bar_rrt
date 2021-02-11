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

def setStiffnessAndDamping(robot, N_ITER, multiplier):

    K = robot.getStiffness()
    D = robot.getDamping()

    Kd = multiplier * K
    Dd = 1 * D

    for k in range(N_ITER):
        stiff = list()
        damp = list()
        for K_start, K_end, D_start, D_end in zip(K, Kd, D, Dd):
            stiff.append(K_start + float(k) / (N_ITER - 1) * (K_end - K_start))
            damp.append(D_start + float(k) / (N_ITER - 1) * (D_end - D_start))
        robot.setStiffness(stiff)
        robot.setDamping(damp)

        # print "Completed: ", float(k) / N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    print "Stiffness of robot is: ", robot.getStiffness()
    print "Damping of robot is: ", robot.getDamping()

def executeCommand(req) :

    # .... do interpolation #
    setStiffnessAndDamping(robot, req.n_iter, req.multiplier)

    return True


if __name__ == '__main__':

    rospy.init_node('xbot_mode_handler')
    if roscpp_init('xbot_mode_handler', []):
        print 'Unable to initialize roscpp node!'
    robot, model = get_robot()
    robot.sense()
    model.syncFrom(robot)

    robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

    s = rospy.Service('xbot_mode/set_stiffness_damping', StiffnessDamping, executeCommand)
    rospy.spin()