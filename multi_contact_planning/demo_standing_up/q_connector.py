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
        self.make_cartesian_interface()

        self.ik_solver = planning.PositionCartesianSolver(self.ci)
        self.planner_client = coman_rising.planner_client()

        self.__trj_sub = rospy.Subscriber('/planner/joint_trajectory', JointTrajectory, self.callback)
        self.__launch = launch

    def callback(self, data):
        self.solution = [data.points[index].positions for index in range(len(data.points))]

    def make_vc_context(self, active_links, quaternions):
        _planner_config = dict()
        _planner_config['state_validity_check'] = ['collisions', 'stability']
        _planner_config['collisions'] = {'type': 'CollisionCheck', 'include_environment': 'true'}
        _planner_config['stability'] = {'type': 'CentroidalStatics',
                                                 'eps': 5 * 1e-2, 'friction_coefficient': 0.71,
                                                 'links': active_links,
                                                 'rotations': quaternions}

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

    def make_cartesian_interface(self):
        # Cartesian task to lift the swing contact
        self.ik_pb = self.make_problem_desc_ik()
        rospy.set_param('/cartesian/problem_description', self.ik_pb)
        self.log_path = '/tmp'
        # get cartesio ros client
        self.ci = pyci.CartesianInterface.MakeInstance('OpenSot',
                                                       self.ik_pb,
                                                       self.model.model,
                                                       self.ik_dt,
                                                       log_path=self.log_path)


        self.ctrl_tasks = []
        for k in self.model.ctrl_points.values():
            self.ctrl_tasks.append(self.ci.getTask(k))

        self.postural = self.ci.getTask('postural')

    def impact_detector(self, lifted_contact, turn, magnitude):

        task = self.ctrl_tasks[lifted_contact]
        detect_bool = 0
        wrench = self.model.ft_map[task.getName()].getWrench()
        # wrench[direction] = 0 # FOR SIMULATION
        direction = [k for k, e in enumerate(self.stance_list[turn][lifted_contact]['ref']['normal']) if e != 0]
        if (wrench[direction] >= magnitude):
            detect_bool = 1

        return detect_bool

    def surface_reacher(self, lifted_contact, turn, force_treshold):

        print 'starting surface reacher...'
        task = self.ctrl_tasks[lifted_contact]
        # velocity desired
        vel_ref = 0.01
        vel_task = vel_ref * (np.append(- np.array(self.stance_list[turn][lifted_contact]['ref']['normal']), [0, 0, 0]))

        print vel_task
        task.enable()
        task.setControlMode(pyci.ControlType.Velocity)
        lambda_value = task.getLambda()
        task.setLambda(0)

        while not self.impact_detector(lifted_contact, turn, force_treshold):

            if not self.impact_detector(lifted_contact, turn, force_treshold):
                task.setVelocityReference(vel_task)

            self.model.robot.sense()
            self.model.model.syncFrom(self.model.robot)
            self.model.f_est.update()


        task.enable()
        task.setControlMode(pyci.ControlType.Position)
        task.setLambda(lambda_value)

        ############################################
        ############################################
        ## Cartesian part
        time_from_reaching = 0.
        CONVERGENCE_TIME = 5.
        UNABLE_TO_SOLVE_MAX = 5
        unable_to_solve = 0
        ci_time = 0.0
        initialize_trj = False
        while task.getTaskState() == pyci.State.Reaching or time_from_reaching <= CONVERGENCE_TIME:
            q = np.hstack((q, cogimon.model.getJointPosition().reshape(cogimon.model.getJointNum(), 1)))

            if not self.ci_solve_integrate(self.ci, cogimon.model, ci_time, self.ik_dt):
                print('Unable to solve!!!')
                unable_to_solve += 1
                print(unable_to_solve)
                # break
                if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
                    print("Maximum number of unable_to_solve reached: ")
                    print(unable_to_solve)
                    return q, False
            else:
                unable_to_solve = 0

            ci_time += self.ik_dt

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

        if normal == [0., 0., 1.]:
            theta = [0, 0, 0]
        elif normal == [-1., 0., 0.]:
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


    def run(self):
        for i in range(2):
        #for i in range(0, len(self.q_list), 1):
            ################################################
            # First Planning Phase to unload swing contact #
            ################################################
            q_start = self.q_list[i]
            self.q_bounder(q_start)

            q_goal = self.q_list[i+1]
            self.q_bounder(q_goal)

            # set all contacts to be active for first planning phase
            active_ind = [ind['ind'] for ind in self.stance_list[i]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  # names
            # self.model.cs.setContactLinks(active_links)

            # set rotation matrix for each contact
            normals = [j['ref']['normal'] for j in self.stance_list[i]]
            [self.model.cs.setContactRotationMatrix(k, j) for k, j in
             zip(active_links, [self.rotation(elem) for elem in normals])]
            rot = [eigenpy.Quaternion(self.rotation(elem)) for elem in normals]

            quat_list = []
            quat = [0., 0., 0., 0.]
            for val in range(0, len(rot)):
                quat[0] = rot[val].x
                quat[1] = rot[val].y
                quat[2] = rot[val].z
                quat[3] = rot[val].w
                map(float, quat)
                quat_list.append(quat)

            contacts = {c: r for c, r in zip(active_links, quat_list)}

            self.planner_client.updateManifold(active_links)

            # publish start and goal states
            self.planner_client.publishStartAndGoal(self.model.model.getEnabledJointNames(), q_start, q_goal)
            raw_input("Start and Goal poses sent")

            # publish the contacts
            optimize_torque = False
            if len(contacts) == 2: # we assume that 2 contacts happen ONLY when on feet
                optimize_torque = True
            self.planner_client.publishContacts(contacts, optimize_torque)
            print contacts
            raw_input("Contacts published")
            rospy.sleep(5)

            self.planner_client.solve(PLAN_MAX_ATTEMPTS=5, planner_type='RRTConnect', plan_time=60, interpolation_time=0.01, goal_threshold=0.5)
            print('First planning phase completed!')


            raw_input("Press to next config")