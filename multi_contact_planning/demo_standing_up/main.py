#!/usr/bin/env python
from cartesian_interface.pyci_all import *
import centroidal_planner.pycpl as cpl
import numpy as np
import xbot_interface.config_options as xbot_opt
import xbot_interface.xbot_interface as xbot
import rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import matlogger2.matlogger as matlog
import yaml
import cogimon
import q_connector
import loader
import os
import gazebo_robot_handler as grh
import eigenpy
from geometry_msgs.msg import Pose

import collections

import sys

from cartesian_interface import pyest
from geometry_msgs.msg import *

def sensors_init(arm_estimation_flag, f_est) :

    ft_map = cogimon.robot.getForceTorque()

    if (arm_estimation_flag) :
        # create force estimator
        indices_wrench = [0, 1, 2]
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
    cpp_argv = []
    if not roscpp.init('standing_up', cpp_argv):
        print 'Unable to initialize roscpp node!'
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('robot_description')
    srdf = rospy.get_param('robot_description_semantic')

    logged_data = []
    log_path = '/tmp'
    ctrl_points = collections.OrderedDict(((0, 'l_ball_tip'), (1, 'r_ball_tip'), (4, 'l_sole'), (5, 'r_sole')))
    # ctrl_points = collections.OrderedDict(((2, 'l_ball_tip_d'), (3, 'r_ball_tip_d'), (4, 'l_sole'), (5, 'r_sole')))


    cogimon = cogimon.Cogimon(urdf, srdf, ctrl_points, logged_data, simulation=True)

    user = os.getenv('ROBOTOLOGY_ROOT')
    q_list0 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/phase0/Paolo/low_knees/qList.txt")
    stances0 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/phase0/Paolo/low_knees/sigmaList.txt")
    q_list1 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/phase1/with_obstacle_3/qList.txt")
    stances1 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/phase1/with_obstacle_3/sigmaList.txt")
    q_list2 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/phase2/double_support/qList.txt")
    stances2 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/phase2/double_support/sigmaList.txt")
    q_list_climbing0 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/wall_climbing/phase0/qList.txt")
    stances_climbing0 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/wall_climbing/phase0/sigmaList.txt")
    q_list_climbing1 = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/wall_climbing/phase1/qList.txt")
    stances_climbing1 = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/wall_climbing/phase1/sigmaList.txt")
    q_list_ladder = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/ladder/yaw_constrained/1/qList.txt")
    stances_ladder = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/ladder/yaw_constrained/1/sigmaList.txt")


    # q_list = q_list0 + q_list1 # + q_list2
    # stances = stances0 + stances1 # + stances2
    q_list = q_list2
    stances = stances2

    # wall climbing
    # q_list = q_list_climbing0 #+ q_list_climbing1
    # stances = stances_climbing0 #+ stances_climbing1

    # ladder
    # q_list = q_list_ladder
    # stances = stances_ladder

    # flag = loader.checkStability(cogimon, stances, q_list)
    # print flag
    # raw_input('click to start interpolation')
    # rospy.sleep(2.)
    # exit()

    if cogimon.simulation and True:
        gzhandler = grh.GazeboRobotHandler()
        gzhandler.set_robot_posture(np.array(q_list2[0])[6:])
        initial_pos = dict()
        cogimon.model.setJointPosition(q_list2[0])
        cogimon.model.update()
        quat = cogimon.model.getPose('base_link').quaternion
        pos = cogimon.model.getPose('base_link').translation
        initial_pos['position'] = pos
        initial_pos['position'][2] += 0.1
        initial_pos['orientation'] = quat
        gzhandler.set_robot_position(initial_pos)

        rospy.sleep(1.)
        wall_pose = dict()

        base_link = gzhandler.get_link_state('base_link', 'world')
        wall_pose['position'] = [2.1 - 1.14631 + base_link.link_state.pose.position.x + 0.5, 0, 0]
        Rz = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0], [np.sin(np.pi/2), np.cos(np.pi/2), 0], [0, 0, 1]])
        quat = eigenpy.Quaternion(Rz)
        wall_pose['orientation'] = [quat.x, quat.y, quat.z, quat.w]
        model_xml = open('/home/luca/.gazebo/models/brick_box_3x1x3/model.sdf').read()
        gzhandler.spawn_sdf_model('wall', model_xml, wall_pose, 'world')

    if cogimon.simulation:
        print 'waiting for xbot_mode...'
        rospy.wait_for_service('xbot_mode/set_stiffness_damping')
        rospy.wait_for_service('xbot_mode/reset_stiffness')
        print 'done.'

    qc = q_connector.Connector(cogimon, q_list, stances)
    # qc.play_all_poses(1)
    # qc.replaySolution('solution.txt')
    # qc.replaySolution('solution_phase0.csv')
    # qc.replaySolution('solution_phase1.csv')
    # qc.replaySolution('solution_phase2.txt')
    # exit()

    qc.run()
    # raw_input('click to see the whole solution')
    # qc.play_solution(1)
    qc.saveSolution()

    # cogimon.robot.sense()
    # cogimon.model.syncFrom(cogimon.robot)




