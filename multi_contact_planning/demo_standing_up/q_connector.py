import cogimon
from cartesian_interface.pyci_all import *
import rospy
import yaml
import numpy as np
from cartesian_interface import pyest
import cartesio_planning.planning as planning
import cartesio_planning.validity_check as vc
from cartesio_planning import NSPG
from trajectory_msgs.msg import JointTrajectory
import eigenpy
import os
import sys
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import roslaunch

# import coman_rising.py from cartesio_planning
user = os.getenv("ROBOTOLOGY_ROOT")
folder = user + "/external/cartesio_planning/examples/python"
sys.path.append(folder)
import coman_rising

class Connector:

    def __init__(self, model, q_list, stance_list, launch=None):
        self.solution = None
        self.rrt_attempts = 0
        self.MAX_RRT_ATTEMPTS = 10  # TODO: PASS AS PARAM
        self.MAX_ERR_NORM = 1e-2  # TODO: PASS AS PARAM
        self.err_norm = 0.0
        self.interp_dt = 0.01

        self.model = model
        self.q_list = q_list
        self.stance_list = stance_list
        self.ik_dt = 0.01
        self.make_cartesian_interface(nspg=True)
        self.make_cartesian_interface(nspg=False)
        # self.ci_ff = pyci.CartesianInterfaceRos("/force_opt")

        self.ik_solver = planning.PositionCartesianSolver(self.ci_nspg)
        self.planner_client = coman_rising.planner_client()
        self.ci_time = 0

        self.__trj_sub = rospy.Subscriber('/planner/joint_trajectory', JointTrajectory, self.callback)
        self.__launch = launch
        self.__solution = list()
        self.__marker_pub = rospy.Publisher('planner/collision_objects', CollisionObject, latch=True, queue_size=10)
        self.__lifted_contact = int()
        self.__lifted_contact_ind = int()
        self.__lifted_contact_link = str()
        self.__counter = 0
        self.__complete_solution = True


    def callback(self, data):
        self.solution = [list(data.points[index].positions) for index in range(len(data.points))]

    def make_vc_context(self, active_links, quaternions, optimize_torque):
        _planner_config = dict()
        _planner_config['state_validity_check'] = ['collisions', 'stability']
        _planner_config['collisions'] = {'type': 'CollisionCheck', 'include_environment': 'true'}
        _planner_config['stability'] = {'type': 'CentroidalStatics',
                                                 'eps': 5 * 1e-2, 'friction_coefficient': 0.71,
                                                 'links': active_links,
                                                 'rotations': quaternions,
                                                 'optimize_torque': optimize_torque,
                                                 'x_lim_cop': [-0.025, 0.065],
                                                 'y_lim_cop': [-0.025, 0.025]}

        vc_context = vc.ValidityCheckContext(yaml.dump(_planner_config), self.model.model)

        return vc_context

    def sensors_init(self, arm_estimation_flag):

        self.f_est = pyest.ForceEstimation(self.model, 0.05)  # 0.05 treshold

        self.ft_map = cogimon.robot.getForceTorque()

        if (arm_estimation_flag):
            # create force estimator
            indices_wrench = [0, 1, 2]
            self.ft_map['l_arm_ft'] = self.f_est.addLink('l_ball_tip', indices_wrench, ['left_arm'])
            self.ft_map['r_arm_ft'] = self.f_est.addLink('r_ball_tip', indices_wrench, ['right_arm'])
            self.f_est.update()


    def make_problem_desc_ik(self):

        # write cartesio config
        ik_cfg = dict()

        ik_cfg['solver_options'] = {'regularization': 1e-3, 'back_end': 'qpoases'}

        ik_cfg['stack'] = [
            self.model.ctrl_points.values() + ['torso']
        ]

        ik_cfg['constraints'] = ['JointLimits']

        ik_cfg['JointLimits'] = {'type': 'JointLimits'}

        for c in self.model.ctrl_points.values():
            ik_cfg[c] = {
                'type': 'Cartesian',
                'base_link': 'torso',
                'distal_link': c,
                'lambda': 1.
            }

        ik_cfg['torso'] = {
            'type': 'Cartesian',
            'distal_link': 'torso',
        }

        ik_str = yaml.dump(ik_cfg)

        return ik_str

    def make_problem_description_ik_NSPG(self):
        # write cartesio config
        ik_cfg = dict()

        ik_cfg['solver_options'] = {'regularization': 1e-3, 'back_end': 'qpoases'}

        ik_cfg['stack'] = [
            self.model.ctrl_points.values(), ['postural']
        ]

        ik_cfg['constraints'] = ['JointLimits']

        ik_cfg['JointLimits'] = {'type': 'JointLimits'}

        ik_cfg['postural'] = {
            'name': 'postural',
            'type': 'Postural',
            'lambda': 0.1,
            # 'weight': 100.0
        }

        for c in self.model.ctrl_points.values():
            if (c == "l_ball_tip" or c == "r_ball_tip"):
                ik_cfg[c] = {
                    'type': 'Cartesian',
                    'distal_link': c,
                    'lambda': 1.,
                    'weight': [1., 1., 1., 0.1, 0.1, 0.1]
                }
            else:
                ik_cfg[c] = {
                    'type': 'Cartesian',
                    'distal_link': c,
                    'lambda': 1.
                }

        ik_str = yaml.dump(ik_cfg)

        return ik_str

    def ci_solve_integrate(self, t):
        if not self.ci.update(t, self.ik_dt):
            return False

        q = self.model.model.getJointPosition()
        qdot = self.model.model.getJointVelocity()

        q += qdot * self.ik_dt

        self.model.model.setJointPosition(q)
        self.model.model.update()

        return True

    def make_cartesian_interface(self, nspg = False):
        # Cartesian task to lift the swing contact
        if nspg:
            self.ik_pb = self.make_problem_description_ik_NSPG()
            rospy.set_param('/cartesian/problem_description_nspg', self.ik_pb)
        else:
            self.ik_pb = self.make_problem_desc_ik()
            rospy.set_param('/cartesian/problem_description', self.ik_pb)

        self.log_path = '/tmp'
        # get cartesio ros client
        if nspg:
            self.ci_nspg = pyci.CartesianInterface.MakeInstance('OpenSot',
                                                           self.ik_pb,
                                                           self.model.model,
                                                           self.ik_dt,
                                                           log_path=self.log_path)
            self.ctrl_tasks_nspg = []
            for k in self.model.ctrl_points.values():
                self.ctrl_tasks_nspg.append(self.ci_nspg.getTask(k))
        else:
            self.ci = pyci.CartesianInterface.MakeInstance('OpenSot',
                                                           self.ik_pb,
                                                           self.model.model,
                                                           self.ik_dt,
                                                           log_path=self.log_path)
            self.ctrl_tasks = []
            for k in self.model.ctrl_points.values():
                self.ctrl_tasks.append(self.ci.getTask(k))

    def impact_detector(self, turn, magnitude):

        task = self.ctrl_tasks[self.__lifted_contact]
        detect_bool = 0
        wrench = self.model.ft_map[task.getName()].getWrench()
        print wrench
        # wrench[direction] = 0 # FOR SIMULATION
        direction = [k for k, e in enumerate(self.stance_list[turn][self.__lifted_contact]['ref']['normal']) if e != 0]
        if (abs(wrench[direction]) >= magnitude):
            detect_bool = 1

        return detect_bool

    def surface_reacher(self, turn, force_treshold):

        print 'starting surface reacher...'
        self.ci.reset(0)
        task = self.ctrl_tasks[self.__lifted_contact_ind]
        # velocity desired
        vel_ref = 0.01
        vel_task = vel_ref * (-np.array(self.stance_list[turn + 1][self.__lifted_contact]['ref']['normal']))
        w_R_t = self.model.model.getPose('torso').linear
        t_vel = np.dot(np.linalg.inv(w_R_t), np.array(vel_task).transpose())
        vel_task = np.append(t_vel, [0, 0, 0])

        print vel_task
        task.enable()
        task.setControlMode(pyci.ControlType.Velocity)
        lambda_value = task.getLambda()
        task.setLambda(0)

        self.ci_time = 0

        while not self.impact_detector(turn, force_treshold):

            task.setVelocityReference(vel_task)
            print task.getPoseReference()[1]

            if not self.ci_solve_integrate(self.ci_time):
                print ('unable to solve')
                break

            q = self.model.model.getJointPosition()
            self.model.robot.setPositionReference(q[6:])
            self.model.robot.move()
            self.ci_time += self.ik_dt

            self.model.robot.sense()
            self.model.model.syncFrom(self.model.robot)
            self.model.f_est.update()
            rospy.sleep(self.ik_dt)

        task.enable()
        task.setControlMode(pyci.ControlType.Position)
        task.setLambda(lambda_value)

        print 'Surface reacher done'

        ############################################
        ############################################

    def q_bounder(self, q):
        for i in range(len(q)):
            if q[i] > self.model.qmax[i]:
                q[i] = self.model.qmax[i]
            elif q[i] < self.model.qmin[i]:
                q[i] = self.model.qmin[i]

    def rotation(self, normal):

        if normal[2] > 0.01:
            theta = [0, 0, 0]
        elif normal[0] < -0.01:
            theta = [0, -np.pi / 2, 0]
        else:
            raise Exception('wrong normal')

        tx, ty, tz = theta

        Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
        Ry = np.array([[np.cos(ty), 0, np.sin(ty)], [0, 1, 0], [-np.sin(ty), 0, np.cos(ty)]])
        Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

        return np.dot(Rz, np.dot(Rx, Ry))

    def play_all_poses(self, num_iter):
        index = 0
        while index < num_iter:
            for i in range(len(self.q_list)):
                self.model.model.setJointPosition(self.q_list[i])
                self.model.model.update()
                self.model.rspub.publishTransforms('ci')

                rospy.sleep(1)
            index = index + 1

    def play_solution(self, iter):
        index = 0
        # rospy.sleep(2.)
        while index < iter:
            for j in range(len(self.__solution)):
                self.model.model.setJointPosition(self.__solution[j])
                self.model.model.update()
                if self.model.simulation:
                    self.model.robot.setPositionReference(self.__solution[j][6:])
                    self.model.robot.move()
                self.model.rspub.publishTransforms('solution')
                rospy.sleep(0.01)
            index = index + 1

    def NSPGsample(self, q_start, q_postural, active_links, quat_list, optimize_torque, timeout):
        # create NSPG
        vc_context = self.make_vc_context(active_links, quat_list, optimize_torque)
        nspg = NSPG.NSPG(self.ik_solver, vc_context)
        
        # set contact position for ik_solver
        self.model.model.setJointPosition(q_start)
        self.model.model.update()
        [self.ci_nspg.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in self.model.ctrl_points.values()]
        self.model.model.setJointPosition(q_postural)
        self.model.model.update()

        print '[NSPG]: start sampling!'
        if not nspg.sample(timeout):
            print '[NSPG]: unable to find a feasible solution!'
            self.saveSolution()
            exit()
        else:
            print '[NSPG]: feasible goal pose found!'
            rospy.sleep(2.)

        # set nspg solution as new goal state
        new_q_goal = nspg.getModel().getJointPosition()
        self.q_bounder(new_q_goal)

        return new_q_goal

    def moveRobot(self, solution, dt):
        for index in range(len(solution)):
            self.model.robot.setPositionReference(solution[index][6:])
            self.model.robot.move()
            rospy.sleep(dt)

    def setClearenceObstacle(self, qstart, qgoal, stance_start):
        if len(stance_start) > 2 and len(stance_start) < 4:
            # find the lifted contact
            lifted_contact = [x for x in list(self.model.ctrl_points.keys()) if
                              x not in [j['ind'] for j in stance_start]][0]
            lifted_contact_link = self.model.ctrl_points[lifted_contact]

            # retrieve start and goal poses for the lifted contact
            self.model.model.setJointPosition(qstart)
            self.model.model.update()
            T = self.model.model.getPose(lifted_contact_link)
            start_pose = T.translation
            self.model.model.setJointPosition(qgoal)
            self.model.model.update()
            T = self.model.model.getPose(lifted_contact_link)
            goal_pose = T.translation

            # if start and goal z-axis position is the same, create the clearence obstacle,
            # otherwise no further action is needed
            clearence = 0.005
            if start_pose[2] - goal_pose[2] < clearence:
                # create Marker
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.header.stamp = rospy.Time.now()

                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = (start_pose[0] + goal_pose[0])/2.
                marker.pose.position.y = (start_pose[1] + goal_pose[1])/2.
                marker.pose.position.z = clearence/2
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1

                marker.scale.x = 0.01
                marker.scale.y = 0.2
                marker.scale.z = clearence

                # create CollisionObject
                co = CollisionObject()
                co.header.frame_id = marker.header.frame_id
                co.id = 'clearence'
                co.header.stamp = rospy.Time.now()
                co.operation = CollisionObject.ADD

                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions.append(marker.scale.x)
                primitive.dimensions.append(marker.scale.y)
                primitive.dimensions.append(marker.scale.z)
                co.primitives.append(primitive)

                pose = Pose()
                pose.position.x = marker.pose.position.x
                pose.position.y = marker.pose.position.y
                pose.position.z = marker.pose.position.z
                pose.orientation.x = marker.pose.orientation.x
                pose.orientation.y = marker.pose.orientation.y
                pose.orientation.z = marker.pose.orientation.z
                pose.orientation.w = marker.pose.orientation.w

                co.primitive_poses.append(pose)

                self.__marker_pub.publish(co)

    def computeStartAndGoal(self, clearence, i):
        # first check if the swing contact has to move on a plane (we assume that this happens when there are
        # three active links). In this case, first we detach the contact from the plane and then we plan to reach
        # the next contact pose
        if len(self.stance_list[i]) == 3: # and i != 3 and i != 2:
            # raw_input('click to compute cartesian trajectory')
            # find the lifted contact
            self.__lifted_contact = [x for x in list(self.model.ctrl_points.keys()) if
                              x not in [j['ind'] for j in self.stance_list[i]]][0]
            self.__lifted_contact_link = self.model.ctrl_points[self.__lifted_contact]
            self.__lifted_contact_ind = self.model.ctrl_points.keys().index(self.__lifted_contact)

            # hardcoded stuff for phase0
            # if i == 4 and self.__complete_solution:
            #     q_start = self.setClearence(i, self.q_list[i], 0.07, 'start')
            # elif i == 4 and not self.__complete_solution:
            #     # self.model.robot.sense()
            #     # self.model.model.syncFrom(self.model.robot)
            #     # q = self.model.model.getJointPosition()
            #     # q[0:5] = self.q_list[i][0:5]
            #     q_start = self.setClearence(i, self.q_list[i], 0.07, 'start')
            # elif i != 4 and self.__complete_solution:
            #     q_start = self.setClearence(i, self.q_list[i], clearence, 'start')
            # elif i != 4 and not self.__complete_solution:
            #     self.model.robot.sense()
            #     self.model.model.syncFrom(self.model.robot)
            #     q = self.model.model.getJointPosition()
            #     # q[0:5] = self.q_list[i][0:5]
            #     q = self.model.model.getJointPosition()
            #     q_start = self.setClearence(i, self.q_list[i], clearence, 'start')

            q_start = self.setClearence(i, self.q_list[i], clearence, 'start')
            self.q_bounder(q_start)

            # q_start = self.setClearence(i, self.q_list[i], clearence, 'start')
            # self.q_bounder(q_start)

            q_goal = self.setClearence(i+1, self.q_list[i+1], clearence, 'goal')
            self.q_bounder(q_goal)

        else:
            if self.__complete_solution:
                q_start = self.q_list[i]
                self.q_bounder(q_start)
            else:
                self.model.robot.sense()
                self.model.model.syncFrom(self.model.robot)
                # q_start = self.model.model.getJointPosition()
                # q_start[0:5] = self.q_list[i][0:5]
                q_start = self.q_list[i]
                self.q_bounder(q_start)
            q_goal = self.q_list[i+1]
            self.q_bounder(q_goal)

        return q_start, q_goal

    def setClearence(self, i, start, clearence, state):
        # find the rotation matrices
        self.model.model.setJointPosition(start)
        self.model.model.update()
        # self.ci.reset(self.ik_dt)

        w_R_t = self.model.model.getPose('torso').linear

        # pick the lifting direction from the stance_list and transform it in torso frame
        normal = list()
        if state == 'start':
            for iterator in range(len(self.stance_list[i - 1])):
                if self.stance_list[i - 1][iterator]['ind'] == self.__lifted_contact:
                    normal = self.stance_list[i - 1][iterator]['ref']['normal']
                    break
        elif state == 'goal' or state == 'touch':
            for iterator in range(len(self.stance_list[i])):
                if self.stance_list[i][iterator]['ind'] == self.__lifted_contact:
                    normal = self.stance_list[i][iterator]['ref']['normal']
                    final = self.stance_list[i][iterator]['ref']['pose']
                    break

        # set clearence: state == start or state == goal -> clearence = clearence
        #                state == touch -> clearence = -model.getPose(lifted_contact).translation * normal
        if state == 'touch':
            p = self.model.model.getPose(self.__lifted_contact_link).translation
            step = np.array(final) - np.array(p)
            clearence = - np.dot(step, [abs(ele) for ele in normal])

        w_p_d = clearence * np.array(normal)
        t_p_d = np.dot(np.linalg.inv(w_R_t), w_p_d.transpose())

        # set pose target for the lifted link
        self.ci.getTask('torso').setPoseReference(self.model.model.getPose('torso'))
        [self.ci.getTask(c).setPoseReference(
            np.dot(self.model.model.getPose('torso').inverse(), self.model.model.getPose(c))) for c in
            self.model.ctrl_points.values()]
        lifted_contact_final_pose = self.ctrl_tasks[self.__lifted_contact_ind].getPoseReference()[0]
        lifted_contact_final_pose.translation += t_p_d
        self.ctrl_tasks[self.__lifted_contact_ind].setPoseTarget(lifted_contact_final_pose, 1.0)

        # solve
        q = np.empty(shape=[self.model.model.getJointNum(), 0])
        while self.ctrl_tasks[self.__lifted_contact_ind].getTaskState() == pyci.State.Reaching:
            q = np.hstack((q, self.model.model.getJointPosition().reshape(self.model.model.getJointNum(), 1)))

            if not self.ci_solve_integrate(self.ci_time):
                print('Unable to solve!!!')
                unable_to_solve += 1
                print(unable_to_solve)
            else:
                unable_to_solve = 0

            self.ci_time += self.ik_dt

        # if self.__complete_solution, save the cartesian solution
        if (state == 'start' or state == 'touch') and self.__complete_solution:
            q_list = list(q.transpose())
            self.__solution = self.__solution + q_list
            self.__counter = self.__counter + len(q_list)

        # else play the solution as soon as it is calculated
        elif (state == 'start' or state == 'touch') and not self.__complete_solution:
            for index in range(np.size(q, 1)):
                self.model.model.setJointPosition(q[:, index])
                self.model.model.update()
                self.model.robot.setPositionReference(q[6:, index])
                self.model.robot.move()
                rospy.sleep(0.01)

        return self.model.model.getJointPosition()


    def run(self):
        s = len(self.q_list) - 1
        i = 0
        while i < s:
            # reset the counter
            self.__counter = 0

            # set start and goal configurations
            [q_start, q_goal] = self.computeStartAndGoal(0.0, i)

            # find active_links for start and goal
            active_ind_goal = [ind['ind'] for ind in self.stance_list[i+1]]
            active_links_goal = [self.model.ctrl_points[j] for j in active_ind_goal]
            active_ind_start = [ind['ind'] for ind in self.stance_list[i]]
            active_links_start = [self.model.ctrl_points[j] for j in active_ind_start]  # names

            # find rotation matrices and quaternions for start and goal active_links
            normals_goal = [j['ref']['normal'] for j in self.stance_list[i+1]]
            normals_start = [j['ref']['normal'] for j in self.stance_list[i]]

            rot_goal = [eigenpy.Quaternion(self.rotation(elem)) for elem in normals_goal]
            quat_list_goal = []
            quat_goal = [0., 0., 0., 0.]
            for val in range(0, len(rot_goal)):
                quat_goal[0] = rot_goal[val].x
                quat_goal[1] = rot_goal[val].y
                quat_goal[2] = rot_goal[val].z
                quat_goal[3] = rot_goal[val].w
                map(float, quat_goal)
                quat_list_goal.append(quat_goal)

            rot_start = [eigenpy.Quaternion(self.rotation(elem)) for elem in normals_start]
            quat_list_start = []
            quat_start = [0., 0., 0., 0.]
            for val in range(0, len(rot_start)):
                quat_start[0] = rot_start[val].x
                quat_start[1] = rot_start[val].y
                quat_start[2] = rot_start[val].z
                quat_start[3] = rot_start[val].w
                map(float, quat_start)
                quat_list_start.append(quat_start)

            # find optimize_torque for start and goal stances
            optimize_torque_goal = False
            if len(active_links_goal) == 2 or i == 2:
                optimize_torque_goal = True
            else:
                optimize_torque_goal = False
            optimize_torque_start = False
            if len(active_links_start) == 2 or i == 2:
                optimize_torque_start = True
            else:
                optimize_torque_start = False

            # check the goal state whether we are adding or removing a contact
            adding = False
            if len(active_links_goal) > len(active_links_start):                             # adding a contact
                adding = True
                self.model.cs.setContactLinks(active_links_start)                            # active_links
                print('active_links_goal set')
                rospy.sleep(2.)
                [self.model.cs.setContactRotationMatrix(k, j) for k, j in
                 zip(active_links_start, [self.rotation(elem) for elem in normals_start])]   # rotations
                print('rotations_goal set')
                rospy.sleep(2.)
                self.model.cs.setOptimizeTorque(optimize_torque_start)                       # optimize_torque
                print('optimize_torque_goal set')
                rospy.sleep(2.)
            else:                                                                            # removing a contact
                self.model.cs.setContactLinks(active_links_goal)                             # active_links
                print('active_links_start set')
                rospy.sleep(2.)
                [self.model.cs.setContactRotationMatrix(k, j) for k, j in
                 zip(active_links_goal, [self.rotation(elem) for elem in normals_goal])]     # rotations
                print('rotations_start set')
                rospy.sleep(2.)
                self.model.cs.setOptimizeTorque(optimize_torque_goal)                        # optimize_torque
                print('optimize_torque_start set')
                rospy.sleep(2.)

            if not self.model.state_vc(q_goal):
                print ['for configuration ', i+1, ' goal pose is not valid, click to compute a new feasible one']
                rospy.sleep(2.)

                if adding:
                    q_goal = self.NSPGsample(q_goal, q_start, active_links_start, quat_list_start, optimize_torque_start, 20.)
                    self.q_list.insert(i + 1, q_goal)
                    self.stance_list.insert(i + 1, self.stance_list[i + 1])
                else:
                    q_goal = self.NSPGsample(q_goal, q_start, active_links_goal, quat_list_goal, optimize_torque_goal, 20.)
                    self.q_list[i+1] = q_goal

            self.planner_client.updateManifold(active_links_start)
            # raw_input('Manifold updated')
            print 'Manifold updated'
            rospy.sleep(2.)

            self.planner_client.publishStartAndGoal(self.model.model.getEnabledJointNames(), q_start, q_goal)
            # raw_input("Start and Goal poses sent")
            print 'Start and Goal poses set'
            if i > 0:
                self.model.replay_model.setJointPosition(self.q_list[i-1])
                self.model.replay_model.update()
                self.model.sol_viz.publishMarkers([], True)
            rospy.sleep(2.)

            if np.linalg.norm(np.array(q_start) - np.array(q_goal)) < 0.05:
                print 'Start and Goal poses are the same, skipping!'
                s = len(self.q_list) - 1
                i = i + 1
                rospy.sleep(2.)
                continue

            # publish the contacts
            contacts = {c: r for c, r in zip(active_links_start, quat_list_start)}
            self.planner_client.publishContacts(contacts, optimize_torque_start)
            print contacts
            # raw_input("Contacts published")
            print 'Contacts published'
            rospy.sleep(2.)

            res = self.planner_client.solve(PLAN_MAX_ATTEMPTS=2, planner_type='RRTConnect', plan_time=60, interpolation_time=0.01, goal_threshold=0.05)
            rospy.sleep(0.5)

            if not res:
                for deleter in range(self.__counter):
                    del self.__solution[-1]
                continue

            if self.__complete_solution:

                self.__solution = self.__solution + self.solution
            else:
                for q in self.solution:
                    self.model.model.setJointPosition(q)
                    self.model.model.update()
                    self.model.robot.setPositionReference(q[6:])
                    self.model.robot.move()
                    rospy.sleep(0.01)

            rospy.sleep(0.5)

            if len(active_links_start) == 3: # and i != 3 and i != 2:
                if self.__complete_solution:
                    dummy_vector = self.setClearence(i+1, q_goal, 0.015, 'touch')
                    self.q_list[i+1] = dummy_vector
                else:
                    self.surface_reacher(i, 20)

            # raw_input("Press to next config")
            s = len(self.q_list) - 1
            i = i + 1
            print 'next config'
            rospy.sleep(2.)

    def saveSolution(self):
        np.savetxt('solution.csv', self.__solution, delimiter=',')
        print 'Solution saved!'

    def replaySolution(self):
        self.__solution = []
        txt = np.loadtxt('solution_phase1.csv', delimiter=',')
        for i in range(np.size(txt, 0)):
            self.__solution.append(txt[i, :])
        self.play_solution(1)
