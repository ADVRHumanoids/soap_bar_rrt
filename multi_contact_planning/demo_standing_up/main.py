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

import sys

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

    rospy.init_node("standing_up")
    np.set_printoptions(precision=3, suppress=True)
    # thr = threading.Thread(target = rospy.spin())
    # thr.start()

    cpp_argv = []
    if not roscpp.init('standing_up', cpp_argv):
        print 'Unable to initialize roscpp node!'
    # define contacts for the ForcePublisher
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('robot_description')
    srdf = rospy.get_param('robot_description_semantic')

    logged_data = []
    log_path = '/tmp'
    ctrl_points = collections.OrderedDict(((0, 'l_ball_tip'), (1, 'r_ball_tip'), (4, 'l_sole'), (5, 'r_sole')))

    cogimon = cogimon.Cogimon(urdf, srdf, ctrl_points, logged_data, simulation=False)

    user = os.getenv('ROBOTOLOGY_ROOT')
    q_list0 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/phase0/qList.txt")
    stances0 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/phase0/sigmaList.txt")
    q_list1 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/phase1/qList.txt")
    stances1 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/phase1/sigmaList.txt")
    q_list2 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/phase2/qList.txt")
    stances2 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/phase2/sigmaList.txt")

    q_list = q_list0 + q_list1 + q_list2
    stances = stances0 + stances1 + stances2

    # qhome = cogimon.model.getRobotState("home")
    # qhome[2] = 0.96
    # q_list.insert(0, qhome)
    # stances.insert(0, stances[0])

    # flag = loader.checkStability(cogimon, stances, q_list)
    # print flag
    # rospy.sleep(2.)
    # exit()

    # if cogimon.simulation:
    #     gzhandler = grh.GazeboRobotHandler()
    #     gzhandler.set_robot_posture(np.array(q_list[4])[6:])
    #
    #     initial_pos = dict()
    #     initial_pos['position'] = q_list[4][0:3]
    #     initial_pos['position'][2] += 0.1
    #     initial_pos['orientation'] = [-0.03, -0.6, -0.03, -0.6]
    #     # initial_pos['orientation'] = [0., 0., 0., 1.]
    #     gzhandler.set_robot_position(initial_pos)

    if cogimon.simulation:
        print 'waiting for xbot_mode...'
        rospy.wait_for_service('xbot_mode/set_stiffness_damping')
        rospy.wait_for_service('xbot_mode/reset_stiffness')
        print 'done.'

    qc = q_connector.Connector(cogimon, q_list1, stances1)
    # qc.play_all_poses(1)
    # qc.replaySolution('solution.csv')
    # qc.replaySolution('solution_phase0.csv')
    # qc.replaySolution('solution_phase1.csv')
    # qc.replaySolution('solution_phase2.csv')
    # exit()

    qc.run()
    # raw_input('click to see the whole solution')
    # qc.play_solution(1)
    # qc.saveSolution()

    # cogimon.robot.sense()
    # cogimon.model.syncFrom(cogimon.robot)




