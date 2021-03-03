#!/usr/bin/env python
import xbot_interface.config_options as xbot_opt
import rospy
from cartesian_interface.pyci_all import *
import loader
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import LinkState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import GetLinkState
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from geometry_msgs.msg import Pose
import yaml
import numpy as np

class GazeboRobotHandler:

    def __init__(self):
        try:
            rospy.init_node('gazebo_robot_handler', anonymous=True)
        except rospy.exceptions.ROSException as e:
            print("GazeboRobotHandler message: Node has already been initialized.")


    def __link_states_callback(self, data):
        self.link_states = data

    def __link_states_listener(self):
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.__link_states_callback)
        rospy.wait_for_message("/gazebo/link_states", LinkStates)

    def __get_link_state_client(self, link_name, reference_frame):
        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            return get_link_state("cogimon::"+link_name, reference_frame)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def __set_link_state_client(self, link_state):
        rospy.wait_for_service('/gazebo/set_link_state')
        try:
            set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
            return set_link_state(link_state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def __set_model_state_client(self, model_state):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            return set_model_state(model_state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def __spawn_sdf_model(self, name, model_xml, pose, reference_frame):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            set_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            return set_sdf_model(name, model_xml, '', pose, reference_frame)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_link_state(self, link_name, reference_frame):
        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            state = get_link_state(link_name, reference_frame)
            return state
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def save_robot_pose(self):

        self.__link_states_listener()

        # link_list = []
        # for elem in self.link_states.name:
        #     if self.__get_link_state_client(elem, 'world').success:
        #         link_state = self.__get_link_state_client(elem, 'world').link_state
        #         link_list.append(link_state)

        self.__link_states_listener()

        organized_link_list = []
        for item in range(len(self.link_states.name)):
            link_dict = dict()
            link_dict['link_name'] = self.link_states.name[item]
            link_dict['reference_frame'] = 'world'
            link_dict['position'] = [self.link_states.pose[item].position.x, self.link_states.pose[item].position.y, self.link_states.pose[item].position.z]
            link_dict['orientation'] = [self.link_states.pose[item].orientation.x, self.link_states.pose[item].orientation.y, self.link_states.pose[item].orientation.z, self.link_states.pose[item].orientation.w]

            organized_link_list.append(link_dict)


        with open('postures.yaml', 'w') as file:
            yaml.dump(organized_link_list, file)


    def load_robot_pose(self, postures_path):

        a_yaml_file = open(postures_path)
        postures = yaml.load(a_yaml_file)

        for elem in postures:

            link_state = LinkState()
            link_state.link_name = elem['link_name']
            link_state.reference_frame = elem['reference_frame']
            link_state.pose.position.x = elem['position'][0]
            link_state.pose.position.y = elem['position'][1]
            link_state.pose.position.z = elem['position'][2]

            link_state.pose.orientation.x = elem['orientation'][0]
            link_state.pose.orientation.y = elem['orientation'][1]
            link_state.pose.orientation.z = elem['orientation'][2]
            link_state.pose.orientation.w = elem['orientation'][3]

            self.__set_link_state_client(link_state)

    def set_robot_position(self, pose):
        model_state = ModelState()
        model_state.model_name = 'cogimon'
        model_state.reference_frame = 'world'
        model_state.pose.position.x = pose['position'][0]
        model_state.pose.position.y = pose['position'][1]
        model_state.pose.position.z = pose['position'][2]

        model_state.pose.orientation.x = pose['orientation'][0]
        model_state.pose.orientation.y = pose['orientation'][1]
        model_state.pose.orientation.z = pose['orientation'][2]
        model_state.pose.orientation.w = pose['orientation'][3]

        self.__set_model_state_client(model_state)

    def set_robot_posture(self, target_posture):

        # define contacts for the ForcePublisher
        opt = xbot_opt.ConfigOptions()

        urdf = rospy.get_param('robot_description')
        srdf = rospy.get_param('robot_description_semantic')

        opt = co.ConfigOptions()
        opt.set_urdf(urdf)
        opt.set_srdf(srdf)
        opt.generate_jidmap()
        opt.set_bool_parameter('is_model_floating_base', True)
        opt.set_string_parameter('model_type', 'RBDL')
        opt.set_string_parameter('framework', 'ROS')
        self.model = xbot.ModelInterface(opt)
        self.robot = xbot.RobotInterface(opt)

        # update from robot
        self.robot.sense()
        self.model.syncFrom(self.robot)

        rspub = pyci.RobotStatePublisher(self.model)
        rspub.publishTransforms('ci')


        start_posture = self.model.getJointPosition()[6:]

        duration = 5

        initial_time = rospy.get_time()
        time = initial_time

        while time < initial_time + duration:
            t_internal = time - initial_time
            tr = t_internal / duration
            tr2 = tr * tr
            tr3 = tr2 * tr
            s = (6.0 * tr3 * tr2 - 15.0 * tr3 * tr + 10.0 * tr3)
            ref = start_posture + (target_posture - start_posture) * s

            self.robot.setPositionReference(ref)
            self.robot.move()
            rospy.sleep(0.01)
            time = time + 0.01

    def spawn_sdf_model(self, name, model_xml, pose, reference_frame):
        msg_pose = Pose()
        msg_pose.position.x = pose['position'][0]
        msg_pose.position.y = pose['position'][1]
        msg_pose.position.z = pose['position'][2]
        msg_pose.orientation.x = pose['orientation'][0]
        msg_pose.orientation.y = pose['orientation'][1]
        msg_pose.orientation.z = pose['orientation'][2]
        msg_pose.orientation.w = pose['orientation'][3]
        self.__spawn_sdf_model(name, model_xml, msg_pose, reference_frame)

if __name__ == '__main__':

    gzh = GazeboRobotHandler()
    q_list = loader.readFromFileConfigs("/home/francesco/advr-superbuild/external/soap_bar_rrt/multi_contact_planning/PlanningData/qList.txt")

    # SET A GIVEN POSE
    gzh.set_robot_posture(np.array(q_list[0])[6:])
    #
    initial_pos = dict()
    initial_pos['position'] = [0.0, 0.0, 0.6]
    initial_pos['orientation'] = [-0.03, -0.8, -0.03, -0.6]

    gzh.set_robot_position(initial_pos)
    # SAVE THE CURRENT POSTURE OF THE ROBOT
    # gzh.save_robot_pose()

    # LOAD A POSTURE THE ROBOT
    # gzh.load_robot_pose('/home/francesco/advr-superbuild/external/soap_bar_rrt/multi_contact_planning/demo_standing_up/postures.yaml')






