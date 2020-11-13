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



# header:
#   seq: 82
#   stamp:
#     secs: 1605290431
#     nsecs: 578617634
#   frame_id: ''
# name: [VIRTUALJOINT_1, VIRTUALJOINT_2, VIRTUALJOINT_3, VIRTUALJOINT_4, VIRTUALJOINT_5, VIRTUALJOINT_6,
#   LHipLat, LHipSag, LHipYaw, LKneePitch, LAnklePitch, LAnkleRoll, RHipLat, RHipSag,
#   RHipYaw, RKneePitch, RAnklePitch, RAnkleRoll, WaistLat, WaistYaw, LShSag, LShLat,
#   LShYaw, LElbj, LForearmPlate, LWrj1, LWrj2, RShSag, RShLat, RShYaw, RElbj, RForearmPlate,
#   RWrj1, RWrj2]
# position: [-0.4918212317705116, 0.01338507386183678, -0.3875078339742292, -0.06102424379276732, 1.7210663461660964, 0.060294108326343895, 0.12958281544340744, -1.9198621771899997, 0.14281721800868286, 1.1283514741417011, -0.872664625997, 0.04894209567833923, 0.13070253987994676, -1.9163422443828646, 0.14383750124650999, 1.1248131969926907, -0.872664625997, 0.04881236721565567, 0.046802848163279585, -0.20104385545690467, -0.06449717362426166, 1.7161233670294302, 0.9457706450900295, -2.1096853646132074, 0.9010016186783658, 1.2774345441377306, 0.6144554783575615, -1.0713673577661587, -0.7645087398915578, -0.2240553026386879, -1.882369119240809, -0.6696843566908492, 1.2492235867617105, -0.08964795479772296]
# velocity: [-2.0025917990383647e-11, -1.265524538295769e-10, -9.274494277028164e-13, -1.2274313130094628e-09, 5.1811197568525474e-11, 1.2727344288789594e-09, -1.0441183607445881e-09, 0.0, -8.782332773253517e-10, 1.4323092802082995e-12, 0.0, -3.5843940421272004e-10, -1.2191565895863033e-09, -2.0467721717846065e-10, -1.0496922916629556e-09, 2.123812113317104e-10, -2.710505431213761e-18, -3.855291280404471e-10, -1.0151955072859549e-09, -6.650477956398363e-10, 1.1661636051370221e-09, 1.329685055170249e-09, 1.0436496289164062e-09, 2.2160141336912327e-10, 1.9969160901275994e-10, 6.685112734195041e-10, 2.0364239162922326e-10, 9.727708777073456e-10, -1.732740865831391e-10, -4.878236659457327e-10, 1.3537904876520906e-10, -8.641472951850663e-10, -1.2232775594944001e-09, -2.727997993457963e-10]
# effort: [0.0, 2.6867397195928788e-14, 630.2975394951001, -8.228741053040265, -86.88465396943762, -7.341275020104556, 0.07358681383012143, 9.676151053835666, 0.781575173219382, 12.239152735732766, 0.20662271681843253, -0.0013257381768504942, -0.04531782222877663, 9.701658706736088, 0.8588366817844286, 12.23922584906536, 0.20662401766840724, -0.0013257381768490735, 1.2250764379147083, -0.25933528635482617, -6.758298111719769, 6.388185204137687, 6.975711399411629, -8.437390433784008, 0.49321911668757423, -0.7880382794309666, 1.82896835316514e-05, -1.4053514455693876, -4.828631874571342, -3.1493798257938908, -9.348597339835482, -0.8065972416055689, -0.2837354058725378, -7.429066645823038e-05]