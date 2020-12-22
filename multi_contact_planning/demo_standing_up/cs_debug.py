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
ctrl_points = collections.OrderedDict(((0, 'l_ball_tip'),(1, 'r_ball_tip'), (4, 'l_sole'), (5, 'r_sole')))

cogimon = cogimon.Cogimon(urdf, srdf, ctrl_points, logged_data, simulation=False)

user = os.getenv('ROBOTOLOGY_ROOT')
q_list = loader.readFromFileConfigs(user + "/external/soap_bar_rrt/multi_contact_planning/phase0+/qList.txt")
stances = loader.readFromFileStances(user + "/external/soap_bar_rrt/multi_contact_planning/phase0+/sigmaList.txt")

# home configuration added for phase0
q_home = [0.0300119, -0.10315, 0.962093, -0, -0.059999, -0,
          0, -0.363826, 0, 0.731245, -0.30742, 0,
          0, -0.363826, 0, 0.731245, -0.30742, 0,
          0, 0,
          0.959931, 0.007266, 0, -1.91986, 0, -0.523599, 0,
          0.959931, -0.007266, 0, -1.91986, 0, -0.523599, 0]
q_list.insert(0, q_home)
stances.insert(0, stances[0])

loader.checkStability(cogimon, stances, q_list)
