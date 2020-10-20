#!/usr/bin/env python
from cartesian_interface.pyci_all import *
from cartesio_acceleration_support.tasks import ForceTask
import centroidal_planner.pycpl as cpl
import numpy as np
import xbot_interface.config_options as xbot_opt
import xbot_interface.xbot_interface as xbot
import rospy
import centroidal_planner.pyforcepub as fp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import matlogger2.matlogger as matlog
from centroidal_planner.srv import setStiffnessDamping
import yaml

from cartesian_interface import pyest
from geometry_msgs.msg import *
def make_problem_desc_ik(ctrl_points):

    # write cartesio config
    ik_cfg = dict()

    ik_cfg['solver_options'] = {'regularization': 1e-4, 'back-end': 'osqp'}

    ik_cfg['stack'] = [
        ctrl_points, ['com'], ['postural']
    ]

    ik_cfg['postural'] = {
        'name': 'postural',
        'type': 'Postural',
        'lambda': 0.01,
    }

    ik_cfg['com'] = {
        'name': 'com',
        'type': 'Com',
        'lambda': 0.01,
    }

    for c in ctrl_points:

        ik_cfg[c] = {
            'type': 'Cartesian',
            'distal_link': c,
            'indices': [0, 1, 2, 3, 4, 5],
            'lambda': .1
        }

    ik_str = yaml.dump(ik_cfg)

    return ik_str


def get_robot():

    np.set_printoptions(precision=3, suppress=True)

    # INIT FOR XBOTCORE
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('xbotcore/robot_description')
    srdf = rospy.get_param('xbotcore/robot_description_semantic')

    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('framework', 'ROS')

    robot = xbot.RobotInterface(opt)
    model = xbot.ModelInterface(opt)

    return robot, model

def sensors_init(arm_estimation_flag, f_est) :


    ft_map = robot.getForceTorque()

    if (arm_estimation_flag) :
        # create force estimator
        indices_wrench = [0,1,2]
        ft_map['l_arm_ft'] = f_est.addLink('l_ball_tip', indices_wrench, ['left_arm'])
        ft_map['r_arm_ft'] = f_est.addLink('r_ball_tip', indices_wrench, ['right_arm'])
        f_est.update()

    return ft_map


def gen_com_planner(contacts, mass, mu) :

    com_pl = cpl.CoMPlanner(contacts, mass)

    # WEIGHTS
    com_pl.SetMu(mu)
    com_pl.SetCoMWeight(1000000000000)  # 100000.0
    com_pl.SetForceWeight(0.)  # 0.0000001
    return com_pl

if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)

    rospy.init_node('standing_up')
    roscpp_init('standing_up', [])
    # define contacts for the ForcePublisher

    #select contacts of COMAN+
    feet_list = ['l_sole', 'r_sole']
    hands_list = ['l_ball_tip', 'r_ball_tip']
    ctrl_points = feet_list + hands_list
    print ctrl_points

    # logger = matlog.MatLogger2('/tmp/feet_on_wall')
    # logger.setBufferMode(matlog.BufferMode.CircularBuffer)
    log_path = '/tmp'
    robot, model = get_robot()
    robot.sense()
    model.syncFrom(robot)

    robot.setControlMode(xbot.ControlMode.Position())

    ik_dt = 0.01
    ik_pb = make_problem_desc_ik(ctrl_points)
    rospy.set_param('/cartesian/problem_description', ik_pb)
    # get cartesio ros client
    ci = pyci.CartesianInterface.MakeInstance('OpenSot',
                                              ik_pb,
                                              model,
                                              ik_dt,
                                              log_path=log_path)

    soles = [ci.getTask(feet_list[0]), ci.getTask(feet_list[1])]
    hands = [ci.getTask(hands_list[0]), ci.getTask(hands_list[1])]

    com = ci.getTask('Com')




    # print 'waiting for xbotcore_impedance...'
    # rospy.wait_for_service('xbotcore_impedance/set_stiffness_damping')
    # print 'done.'
    #
    # f_est = pyest.ForceEstimation(model, 0.05) # 0.05 treshold
    # ft_map = sensors_init(arm_estimation_flag=True, f_est=f_est)
