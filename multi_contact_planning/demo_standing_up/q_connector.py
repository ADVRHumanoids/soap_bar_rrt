import cogimon
from cartesian_interface.pyci_all import *
import rospy
import yaml
import numpy as np
from cartesian_interface import pyest
import manifold
import cartesio_planning.planning as planning
import cartesio_planning.validity_check as vc
from cartesio_planning import NSPG
from cartesio_planning.msg import SetContactFrames
import eigenpy
import os
import sys

# import coman_rising.py from cartesio_planning
user = os.getenv("ROBOTOLOGY_ROOT")
folder = user + "/external/cartesio_planning/examples/python"
sys.path.append(folder)
import coman_rising

class Connector:

    def __init__(self, model, q_list, stance_list):
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

        self.contacts_pub = rospy.Publisher('/contacts', SetContactFrames, queue_size=10, latch=True)
        self.planner_client = coman_rising.planner_client()

    # def make_vc_context(self, active_links, quaternions):
    #     _planner_config = dict()
    #     _planner_config['state_validity_check'] = ['collisions', 'centroidal_statics']
    #     _planner_config['collisions'] = {'type': 'CollisionCheck', 'include_environment': 'true'}
    #     _planner_config['centroidal_statics'] = {'type': 'CentroidalStatics',
    #                                              'eps': 5 * 1e-2, 'friction_coefficient': 0.71,
    #                                              'links': active_links,
    #                                              'rotations': quaternions}
    #
    #     vc_context = vc.ValidityCheckContext(yaml.dump(_planner_config), self.model.model)
    #
    #     return vc_context

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

        ik_cfg['com'] = {
            'name': 'com',
            'type': 'Com',
            'lambda': 0.01,
        }

        for c in self.model.ctrl_points.values():
            if (c == "l_ball_tip" or c == "r_ball_tip"):
                ik_cfg[c] = {
                    'type': 'Cartesian',
                    'distal_link': c,
                    # 'indices': [0, 1, 2],
                    'lambda': 0.1,
                    'weight': [1., 1., 1., 0.1, 0.1, 0.1]
                }
            else:
                ik_cfg[c] = {
                    'type': 'Cartesian',
                    'distal_link': c,
                    'lambda': 0.1
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
        for i in range(0, len(self.q_list), 2):
        # for i in range(0, 6, 2):
            ################################################
            # First Planning Phase to unload swing contact #
            ################################################
            q_start = self.q_list[i]
            # self.model.model.setJointPosition(q_start)
            # self.model.model.update()
            self.q_bounder(q_start)
            # self.model.ps.update()
            # self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())
            # self.model.rspub.publishTransforms('')

            # poses = [self.model.model.getPose(link) for link in self.model.ctrl_points.values()]
            # print 'START POSES'
            # for printer in range(len(poses)):
            #     print 'active_link: ', self.model.ctrl_points.values()[printer], '\n'
            #     print poses[printer].matrix()

            q_goal = self.q_list[i+1]
            self.q_bounder(q_goal)

            self.planner_client.publishStartAndGoal(self.model.model.getEnabledJointNames(), q_start, q_goal)
            # self.model.model.setJointPosition(q_goal)
            # self.model.model.update()
            # self.model.ps.update()
            # self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # poses = [self.model.model.getPose(link) for link in self.model.ctrl_points.values()]
            # print 'GOAL POSES'
            # for printer in range(len(poses)):
            #     print 'active_link: ', self.model.ctrl_points.values()[printer], '\n'
            #     print poses[printer].matrix()

            # set all contacts to be active for first planning phase
            active_ind = [ind['ind'] for ind in self.stance_list[i]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  #names
            self.model.cs.setContactLinks(active_links)

            # set rotation matrix for each contact
            normals = [j['ref']['normal'] for j in self.stance_list[i]]
            [self.model.cs.setContactRotationMatrix(k, j) for k, j in zip(active_links,  [self.rotation(elem) for elem in normals])]
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

            # create a dictionary of contacts and rotations for the planner_client
            contacts = {c: r for c, r in zip(active_links, quat_list)}
            print contacts
            self.planner_client.publishContacts(contacts)

            # print 'FIRST PLANNING PHASE CS'
            # for c in self.model.cs.getContactLinks():
            #    print 'active link: \n', c, '\n rotation: \n', self.model.cs.getContactFrame(c)

            # os.system('cls' if os.name == 'nt' else 'clear')

            # index = 0
            # if np.linalg.norm(np.array(q_start) - np.array(q_goal)) > 0.05:
            #     solution = None
            #     print 'start first planning phase ...'
            #     while solution is None and index < self.MAX_RRT_ATTEMPTS:
            #         index = index + 1
            #
            #         if index == self.MAX_RRT_ATTEMPTS:
            #             error("Unable to find a feasible plan!")
            #
            #         rot = [eigenpy.Quaternion(self.rotation(elem)) for elem in normals]
            #         quat_list = []
            #         quat = [0., 0., 0., 0.]
            #         for val in range(0, len(rot)):
            #             quat[0] = rot[val].x
            #             quat[1] = rot[val].y
            #             quat[2] = rot[val].z
            #             quat[3] = rot[val].w
            #             map(float, quat)
            #             quat_list.append(quat)
            #
            #         def is_state_valid(q):
            #             self.model.model.setJointPosition(q)
            #             self.model.model.update()
            #             return self.is_model_state_valid()
            #
            #         vc_context = self.make_vc_context(active_links, quat_list)
            #         [self.ci.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in
            #          self.model.ctrl_points.values()]
            #         self.nspg = NSPG.NSPG(self.ik_solver, vc_context)
            #
            #         solution, error = self.model.plan_step(q_start, q_goal, planner_type='RRT', timeout=120.0, threshold = 0.1)
            #         print 'done!'
            #     print 'start interpolation ...'
            #     solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.01)
            #     print 'done!'

                #     # solution_interp = np.loadtxt('solution.csv', delimiter=',')
                #     for val in range(np.size(solution_interp, 1)):
                #         self.model.replay_model.setJointPosition(solution_interp[:, val])
                #         self.model.replay_model.update()
                #         self.model.model.setJointPosition(solution_interp[:, val])
                #         self.model.model.update()
                #         self.model.ps.update()
                #         self.model.planner_viz.publishMarkers(self.model.ps.getCollidingLinks())
                #         rospy.sleep(self.ik_dt)
                #
                    # project the interpolated trajectory onto the maninfold
                # self.make_cartesian_interface()
            #     ci_time = 0
            #     UNABLE_TO_SOLVE_MAX = 5.
            #     q_first_plan = np.empty(shape=[self.model.model.getJointNum(), 0])
            #     self.model.model.setJointPosition(self.q_list[i])
            #     self.model.model.update()
            #     [self.ci.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in self.model.ctrl_points.values()]
            #     index = 0
            #     for index in range(np.size(solution_interp, 1)):
            #         qi = solution_interp[:,  index]
            #         jmap = self.model.model.eigenToMap(qi)
            #         self.postural.setReferencePosture(jmap)
            #         q_first_plan = np.hstack((q_first_plan, self.model.model.getJointPosition().reshape(self.model.model.getJointNum(), 1)))
            #
            #         if not self.ci_solve_integrate(ci_time):
            #             print('Unable to solve!!!')
            #             unable_to_solve += 1
            #             print(unable_to_solve)
            #             # break
            #             if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
            #                 print("Maximum number of unable_to_solve reached: ")
            #                 print(unable_to_solve)
            #         else:
            #             unable_to_solve = 0
            #
            #         ci_time += self.ik_dt
            #
            #     for val in range(np.size(q_first_plan, 1)):
            #         self.model.replay_model.setJointPosition(q_first_plan[:, val])
            #         self.model.replay_model.update()
            #         self.model.model.setJointPosition(q_first_plan[:, val])
            #         self.model.model.update()
            #         self.model.ps.update()
            #         self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())
            #
            #         # sending trajectories to robot
            #         # self.model.robot.setPositionReference(q_first_plan[6:, val])
            #         # self.model.robot.move()
            #
            #         rospy.sleep(self.ik_dt)
            #     print 'done!'
            #
            # else:
            #     print '1st planning phase skipped for pose ', i ,' : start = goal'
            #     rospy.sleep(2)

            # np.savetxt('solution.csv', solution_interp, delimiter=',')

            # find the lifted contact
            self.model.model.setJointPosition(self.q_list[i+1])
            self.model.model.update()

            # self.make_cartesian_interface()
            [self.ci.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in self.model.ctrl_points.values()]
            lifted_contact = [x for x in list(self.model.ctrl_points.keys()) if
                              x not in [j['ind'] for j in self.stance_list[i + 1]]][0]

            lifted_contact_ind = self.model.ctrl_points.keys().index(lifted_contact)

            # set active contacts for second planning phase
            active_ind = [ind['ind'] for ind in self.stance_list[i + 1]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  # names
            self.model.cs.setContactLinks(active_links)

            normals = [j['ref']['normal'] for j in self.stance_list[i + 1]]
            [self.model.cs.setContactRotationMatrix(k, j) for k, j in zip(active_links, [self.rotation(elem) for elem in normals])]

            # if i+2 is unstable on 3 contact, find a new intermediate feasible pose
            if not self.model.state_vc(self.q_list[i+2]):

                #########################################################
                # Intermediate Planning Phase to move the swing contact #
                #########################################################

                print 'configuration ', i+2, ' is unstable on 3 contacts, finding a new one...'
                rospy.sleep(2)

                # contacts = SetContactFrames()
                # contacts.action = SetContactFrames.SET
                # contacts.frames_in_contact = active_links
                # contacts.rotations = [eigenpy.Quaternion(self.rotation(elem)) for elem in normals]
                # contacts.friction_coefficient = 0.5 * np.sqrt(2)
                #
                # self.contacts_pub.publish(contacts)
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

                self.vc_context = self.make_vc_context(active_links, quat_list)
                self.nspg = NSPG.NSPG(self.ik_solver, self.vc_context)

                self.model.model.setJointPosition(self.q_list[i+2])
                self.model.model.update()
                [self.ci.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in self.model.ctrl_points.values()]

                # if NSPG finds a solution, assign it as goal configuration
                if not self.nspg.sample(10.0):
                    error('[NSPG]: Unable to find a solution!')
                else:
                    print '[NSPG]: Feasible goal pose found!'
                    rospy.sleep(2)

                q_goal = self.nspg.getModel().getJointPosition()
                self.q_bounder(q_goal)
                self.model.model.setJointPosition(q_goal)
                self.model.model.update()
                self.model.ps.update()
                self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

                self.model.model.setJointPosition(self.q_list[i+2])
                self.model.model.update()
                self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

                # set start state
                q_start = self.q_list[i+1]
                self.q_bounder(q_start)
                self.model.model.setJointPosition(q_start)
                self.model.model.update()
                self.model.ps.update()
                self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

                solution = None
                index = 0
                print 'starting intermediate planning phase ...'
                while solution is None and index < self.MAX_RRT_ATTEMPTS:
                    index = index + 1

                    if index == self.MAX_RRT_ATTEMPTS:
                        raise Exception("Unable to find a feasible plan!")

                    solution, error = self.model.plan_step(q_start, q_goal, lifted_contact, planner_type='RRTConnect', timeout=60.0, threshold = 0.1)
                solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0)

                print 'done!'

                # np.savetxt('solution2.csv', solution_interp, delimiter=',')
                # solution_interp = np.loadtxt('solution2.csv', delimiter=',')

                ci_time = 0
                UNABLE_TO_SOLVE_MAX = 5.
                q_second_plan = np.empty(shape=[self.model.model.getJointNum(), 0])
                self.model.model.setJointPosition(self.q_list[i+1])
                self.model.model.update()
                [self.ci.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in self.model.ctrl_points.values()]
                index = 0
                for index in range(np.size(solution_interp, 1)):
                    qi = solution_interp[:, index]
                    jmap = self.model.model.eigenToMap(qi)
                    self.postural.setReferencePosture(jmap)
                    self.model.replay_model.setJointPosition(solution_interp[:, index])
                    self.model.replay_model.update()
                    self.ctrl_tasks[lifted_contact_ind].setPoseReference(self.model.replay_model.getPose(self.model.ctrl_points[lifted_contact]))

                    if not self.ci_solve_integrate(ci_time):
                        print('Unable to solve!!!')
                        unable_to_solve += 1
                        print(unable_to_solve)
                        # break
                        if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
                            print("Maximum number of unable_to_solve reached: ")
                            print(unable_to_solve)
                    else:
                        q_second_plan = np.hstack((q_second_plan, self.model.model.getJointPosition().reshape(
                            self.model.model.getJointNum(), 1)))
                        unable_to_solve = 0

                    ci_time += self.ik_dt

                # for val in range(np.size(solution_interp, 1)):
                #     self.model.replay_model.setJointPosition(solution_interp[:, val])
                #     self.model.replay_model.update()
                #     self.model.model.setJointPosition(solution_interp[:, val])
                #     self.model.model.update()
                #     self.model.ps.update()
                #     self.model.planner_viz.publishMarkers(self.model.ps.getCollidingLinks())
                #     rospy.sleep(self.ik_dt)

                # project solution_interp onto the manifold
                for val in range(np.size(q_second_plan, 1)):
                    self.model.replay_model.setJointPosition(q_second_plan[:, val])
                    self.model.replay_model.update()
                    self.model.model.setJointPosition(q_second_plan[:, val])
                    self.model.model.update()
                    self.model.ps.update()
                    self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())

                    # sending trajectories to robot
                    # self.model.robot.setPositionReference(q_second_plan[6:, val])
                    # self.model.robot.move()

                    rospy.sleep(self.ik_dt)
                print 'done!'


            ###########################################
            # Last Planning Phase to load the contact #
            ###########################################

            # set start state
            q_start = self.model.model.getJointPosition()
            self.q_bounder(q_start)
            self.model.ps.update()
            self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # set goal state
            q_goal = self.q_list[i + 2]
            self.q_bounder(q_goal)
            self.model.model.setJointPosition(q_goal)
            self.model.model.update()
            self.model.ps.update()
            self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # set all links to be active
            active_ind = [ind['ind'] for ind in self.stance_list[i + 2]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  # names
            self.model.cs.setContactLinks(active_links)

            normals = [j['ref']['normal'] for j in self.stance_list[i + 2]]
            [self.model.cs.setContactRotationMatrix(k, j) for k, j in
             zip(active_links, [self.rotation(elem) for elem in normals])]

            solution = None
            index = 0
            print 'starting intermediate planning phase ...'
            while solution is None and index < self.MAX_RRT_ATTEMPTS:
                index = index + 1

                if index == self.MAX_RRT_ATTEMPTS:
                    raise Exception("Unable to find a feasible plan!")

                solution, error = self.model.plan_step(q_start, q_goal, planner_type='RRTConnect',
                                                       timeout=60.0, threshold=0.1)
            solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.1)

            print 'done!'

            # np.savetxt('solution2.csv', solution_interp, delimiter=',')
            # solution_interp = np.loadtxt('solution2.csv', delimiter=',')

            ci_time = 0
            UNABLE_TO_SOLVE_MAX = 5.
            q_second_plan = np.empty(shape=[self.model.model.getJointNum(), 0])
            self.model.model.setJointPosition(self.q_list[i + 1])
            self.model.model.update()
            [self.ci.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in self.model.ctrl_points.values()]
            index = 0
            for index in range(np.size(solution_interp, 1)):
                qi = solution_interp[:, index]
                jmap = self.model.model.eigenToMap(qi)
                self.postural.setReferencePosture(jmap)


                if not self.ci_solve_integrate(ci_time):
                    print('Unable to solve!!!')
                    unable_to_solve += 1
                    print(unable_to_solve)
                    # break
                    if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
                        print("Maximum number of unable_to_solve reached: ")
                        print(unable_to_solve)
                else:
                    q_second_plan = np.hstack(
                        (q_second_plan, self.model.model.getJointPosition().reshape(self.model.model.getJointNum(), 1)))
                    unable_to_solve = 0

                ci_time += self.ik_dt


            else:
                print 'pose ', i+2, ' is stable on 3 contact, no intermediate pose is needed!'
                rospy.sleep(2)

                ############################################################
                # Second Planning Phase to move and load the swing contact #
                ############################################################

                # set start state
                q_start = self.q_list[i + 1]
                self.q_bounder(q_start)
                self.model.model.setJointPosition(q_start)
                self.model.model.update()
                self.model.ps.update()
                self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

                # set goal state
                q_goal = self.q_list[i + 2]
                self.q_bounder(q_goal)
                self.model.model.setJointPosition(q_goal)
                self.model.model.update()
                self.model.ps.update()
                self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

                solution = None
                index = 0
                print 'starting intermediate planning phase ...'
                while solution is None and index < self.MAX_RRT_ATTEMPTS:
                    index = index + 1

                    if index == self.MAX_RRT_ATTEMPTS:
                        raise Exception("Unable to find a feasible plan!")

                    solution, error = self.model.plan_step(q_start, q_goal, lifted_contact, planner_type='RRTConnect',
                                                           timeout=60.0, threshold=0.0005)
                solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.1)

                print 'done!'

                # np.savetxt('solution2.csv', solution_interp, delimiter=',')
                # solution_interp = np.loadtxt('solution2.csv', delimiter=',')

                ci_time = 0
                UNABLE_TO_SOLVE_MAX = 5.
                q_second_plan = np.empty(shape=[self.model.model.getJointNum(), 0])
                self.model.model.setJointPosition(self.q_list[i + 1])
                self.model.model.update()
                [self.ci.getTask(c).setPoseReference(self.model.model.getPose(c)) for c in self.model.ctrl_points.values()]
                index = 0
                for index in range(0, np.size(solution_interp, 1)):
                    qi = solution_interp[:, index]
                    jmap = self.model.model.eigenToMap(qi)
                    self.postural.setReferencePosture(jmap)
                    self.model.replay_model.setJointPosition(solution_interp[:, index])
                    self.model.replay_model.update()
                    self.ctrl_tasks[lifted_contact_ind].setPoseReference(
                        self.model.replay_model.getPose(self.model.ctrl_points[lifted_contact]))

                    if not self.ci_solve_integrate(ci_time):
                        print('Unable to solve!!!')
                        unable_to_solve += 1
                        print(unable_to_solve)
                        # break
                        if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
                            print("Maximum number of unable_to_solve reached: ")
                            print(unable_to_solve)
                    else:
                        q_second_plan = np.hstack(
                            (q_second_plan,
                             self.model.model.getJointPosition().reshape(self.model.model.getJointNum(), 1)))
                        unable_to_solve = 0

                    ci_time += self.ik_dt

                for val in range(np.size(solution_interp, 1)):
                    self.model.replay_model.setJointPosition(solution_interp[:, val])
                    self.model.replay_model.update()
                    self.model.model.setJointPosition(solution_interp[:, val])
                    self.model.model.update()
                    self.model.ps.update()
                    self.model.planner_viz.publishMarkers(self.model.ps.getCollidingLinks())
                    rospy.sleep(self.ik_dt)

                # project solution_interp onto the manifold
                for val in range(np.size(q_second_plan, 1)):
                    self.model.replay_model.setJointPosition(q_second_plan[:, val])
                    self.model.replay_model.update()
                    self.model.model.setJointPosition(q_second_plan[:, val])
                    self.model.model.update()
                    self.model.ps.update()
                    self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())

                    # sending trajectories to robot
                    # self.model.robot.setPositionReference(q_second_plan[6:, val])
                    # self.model.robot.move()

                    rospy.sleep(self.ik_dt)
                print 'done!'



            # Surface reacher
            # self.surface_reacher(lifted_contact, i+2, 20)
