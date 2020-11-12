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
import cogimon
import q_connector
import loader
import os
import gazebo_robot_handler as grh

import collections

from cartesian_interface import pyest
from geometry_msgs.msg import *

def sensors_init(arm_estimation_flag, f_est) :

    ft_map = cogimon.robot.getForceTorque()

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
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('robot_description')
    srdf = rospy.get_param('robot_description_semantic')

    logged_data = []
    log_path = '/tmp'
    ctrl_points = collections.OrderedDict(((0, 'l_ball_tip'),(1, 'r_ball_tip'), (4, 'l_sole'), (5, 'r_sole')))
    # ctrl_points = {0: 'l_ball_tip', 1: 'r_ball_tip', 4: 'l_sole', 5: 'r_sole'}

    cogimon = cogimon.Cogimon(urdf, srdf, ctrl_points, logged_data)

    user = os.getenv('ROBOTOLOGY_ROOT')
    q_list = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/PlanningData/qList.txt")
    stances = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/PlanningData/sigmaList.txt")

    gzhandler = grh.GazeboRobotHandler()
    gzhandler.set_robot_posture(np.array(q_list[0])[6:])
    #
    initial_pos = dict()
    initial_pos['position'] = [-0.0519934, -0.00367742, 0.63]
    initial_pos['orientation'] = [-0.03, -0.8, -0.03, -0.6]
    gzhandler.set_robot_position(initial_pos)
    # flags = loader.checkStability(cogimon, stances, q_list)
    # print flags
    # raw_input('diocane')
    qc = q_connector.Connector(cogimon, q_list, stances)
    qc.run()
    # logger = matlog.MatLogger2('/tmp/feet_on_wall')
    # logger.setBufferMode(matlog.BufferMode.CircularBuffer)
    # log_path = '/tmp'

    # cogimon.robot.sense()
    # cogimon.model.syncFrom(cogimon.robot)


