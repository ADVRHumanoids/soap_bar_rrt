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
        self.ik_dt = 0.001
        self.manifold = manifold

        self.contacts_pub = rospy.Publisher('contacts', SetContactFrames, queue_size=10, latch=True)

    def make_vc_context(self):
        _planner_config = dict()
        _planner_config['state_validity_check'] = ['collisions', 'centroidal_statics']
        _planner_config['collisions'] = {'type': 'CollisionCheck', 'include_environment': 'true'}
        _planner_config['centroidal_statics'] = {'type': 'CentroidalStatics',
                                                 'eps': 1e-2, 'friction_coefficient': 0.71,
                                                 'links': ['l_sole', 'r_sole', 'l_ball_tip','r_ball_tip']}

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

        ik_cfg['solver_options'] = {'regularization': 10, 'back-end': 'qpoases'}

        ik_cfg['stack'] = [
            self.model.ctrl_points.values(), ['com'], ['postural']
        ]

        ik_cfg['constraints'] = ['JointLimits']

        ik_cfg['JointLimits'] = {'type': 'JointLimits'}

        ik_cfg['postural'] = {
            'name': 'postural',
            'type': 'Postural',
            'lambda': 0.1,
            'weight': 10.0
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
                    'lambda': 0.1
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
        qddot = self.model.model.getJointAcceleration()

        q += qdot * self.ik_dt + 0.5 * qddot * self.ik_dt ** 2
        qdot += qddot * self.ik_dt

        self.model.model.setJointPosition(q)
        self.model.model.setJointVelocity(qdot)
        self.model.model.update()

        return True

    def make_cartesian_interface(self):
        # Cartesian task to lift the swing contact
        self.ik_pb = self.make_problem_desc_ik()
        # rospy.set_param('/cartesian/problem_description', self.ik_pb)
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

        self.com = self.ci.getTask('com')
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

    def run(self):
        index = 0

        for i in range(0, len(self.q_list), 2):
            # First planning phase to unload swing contact
            q_start = self.q_list[i]
            self.model.model.setJointPosition(q_start)
            self.model.model.update()
            self.q_bounder(q_start)
            self.model.ps.update()
            self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

            self.model.model.setJointPosition(q_start)
            self.model.model.update()
            self.model.ps.update()
            self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

            q_goal = self.q_list[i+1]
            self.q_bounder(q_goal)
            self.model.model.setJointPosition(q_goal)
            self.model.model.update()
            self.model.ps.update()
            self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # set all contacts to be active for first planning phase
            active_ind = [ind['ind'] for ind in self.stance_list[i]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  #names
            self.model.cs.setContactLinks(active_links)

            # set rotation matrix for each contact
            normals = [j['ref']['normal'] for j in self.stance_list[i]]

            # print normals
            [self.model.cs.setContactRotationMatrix(k,j) for k, j in zip(active_links,  [self.rotation(elem) for elem in normals])]

            # solution = None
            # print 'start first planning phase ...'
            # while solution is None and index < self.MAX_RRT_ATTEMPTS:
            #     index = index + 1
            #
            #     if index == self.MAX_RRT_ATTEMPTS:
            #         error("Unable to find a feasible plan!")
            #
            #     solution, error = self.model.plan_step(q_start, q_goal, planner_type='RRTConnect', timeout=60.0)
            #
            # print 'done!'
            # print 'start interpolation ...'
            # solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.01)
            # print 'done!'
            #
            # np.savetxt('solution.csv', solution_interp, delimiter=',')

            solution_interp = np.loadtxt('solution.csv', delimiter=',')

            print (np.size(solution_interp,1))
            print (np.size(solution_interp,0))

            print 'replicating trajectory (1/4) ...'
            for val in range(np.size(solution_interp, 1)):
                self.model.replay_model.setJointPosition(solution_interp[:, val])
                self.model.replay_model.update()
                self.model.model.setJointPosition(solution_interp[:, val])
                self.model.model.update()
                self.model.ps.update()
                self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())

                # sending trajectories to robot
                self.model.robot.setPositionReference(solution_interp[6:, val])
                self.model.robot.move()

                rospy.sleep(self.ik_dt)
            print 'done!'

            # sync model and robot
            self.model.robot.sense()
            self.model.model.syncFrom(self.model.robot)

            # get active contacts from 'stance_list' to retrieve lifted contact
            # hypo: there must be a lifted contact
            lifted_contact = [x for x in list(self.model.ctrl_points.keys()) if
                              x not in [j['ind'] for j in self.stance_list[i + 1]]][0]

            lifted_contact_ind = self.model.ctrl_points.keys().index(lifted_contact)

            self.make_cartesian_interface()
            self.ik_solver = planning.PositionCartesianSolver(self.ci)
            self.vc_context = self.make_vc_context()
            self.NSPG = NSPG.NSPG(self.ik_solver, self.vc_context)

            scale = 0.03
            lifted_contact_final_pose = self.ctrl_tasks[lifted_contact_ind].getPoseReference()[0]
            lifted_contact_final_pose.translation = lifted_contact_final_pose.translation + scale * np.array(self.stance_list[i][lifted_contact_ind]['ref']['normal'])
            self.ctrl_tasks[lifted_contact_ind].setPoseTarget(lifted_contact_final_pose, 1.0)

            # w = np.eye(6)*1
            # w[3, 3] = 0.001
            # w[4, 4] = 0.001
            # w[5, 5] = 0.001
            # self.ctrl_tasks[self.model.ctrl_points.keys().index(0)].setWeight(w)
            # self.ctrl_tasks[self.model.ctrl_points.keys().index(1)].setWeight(w)
            # self.ctrl_tasks[self.model.ctrl_points.keys().index(4)].setWeight(np.eye(6))
            # self.ctrl_tasks[self.model.ctrl_points.keys().index(5)].setWeight(np.eye(6))
            #
            self.postural.setReferencePosture(self.model.model.eigenToMap(self.q_list[i+1]))

            # Cartesian part
            ci_time = 0.0
            UNABLE_TO_SOLVE_MAX = 5.
            q = np.empty(shape=[self.model.model.getJointNum(), 0])

            print 'starting cartesian trajectory ...'
            while self.ctrl_tasks[lifted_contact_ind].getTaskState() == pyci.State.Reaching:
                q = np.hstack((q, self.model.model.getJointPosition().reshape(self.model.model.getJointNum(), 1)))

                if not self.ci_solve_integrate(ci_time):
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
            print 'done!'

            print 'replicating trajectory(2/4) ...'
            for j in range(np.size(q, 1)):
                self.model.replay_model.setJointPosition(q[:, j])
                self.model.replay_model.update()
                self.model.model.setJointPosition(q[:, j])
                self.model.model.update()
                self.model.ps.update()
                self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())

            print 'done!'

            # Check whether the final pose of the cartesian trajectory is in collision.
            # is so, the NSPG finds a new collision-free and stable robot configuration and
            # a planner finds a feasible trajectory to connect q_list[i+1] and the configuration
            # coming from the NSPG
            if self.model.state_vc(q[:, -1]):
                for j in range(np.size(q, 1)):
                    # sending trajectories to robot
                    self.model.robot.setPositionReference(q[6:, j])
                    self.model.robot.move()
                    rospy.sleep(self.ik_dt)

                self.model.robot.sense()
                self.model.model.syncFrom(self.model.robot)

            else:
                # update contact list and rotation matrices through '/contact' topic
                active_ind = [ind['ind'] for ind in self.stance_list[i+1]]
                active_links = [self.model.ctrl_points[j] for j in active_ind]

                normals = [j['ref']['normal'] for j in self.stance_list[i+1]]

                contacts = SetContactFrames()
                contacts.action = SetContactFrames.SET
                contacts.frames_in_contact = active_links
                contacts.rotations = [eigenpy.Quaternion(self.rotation(elem)) for elem in normals]
                contacts.friction_coefficient = 0.5 * np.sqrt(2)

                self.contacts_pub.publish(contacts)

                # sample
                if not self.NSPG.sample(10.0):
                    print('Error: Unable to find a collision free and stable pose!')
                    exit()

                # if NSPG finds a feasible pose, start planning
                q_goal = self.NSPG.getModel().getJointPosition()
                self.q_bounder(q_goal)
                self.model.model.setJointPosition(q_goal)
                self.model.model.update()
                self.model.ps.update()
                self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

                q_start = self.q_list[i + 1]
                self.q_bounder(q_start)
                self.model.model.setJointPosition(q_start)
                self.model.model.update()
                self.model.ps.update()
                self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

                solution = None
                while solution is None and index < self.MAX_RRT_ATTEMPTS:
                    index = index + 1

                    if index == self.MAX_RRT_ATTEMPTS:
                        error("Unable to find a feasible plan!")

                    solution, error = self.model.plan_step(q_start, q_goal, lifted_contact, planner_type='RRTConnect', timeout=60.0)

                solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.01)

                for val in range(np.size(solution_interp, 1)):
                    self.model.replay_model.setJointPosition(solution_interp[:,val])
                    self.model.replay_model.update()
                    self.model.model.setJointPosition(solution_interp[:,val])
                    self.model.model.update()
                    self.model.ps.update()
                    self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())

                    # sending trajectories to robot
                    self.model.robot.setPositionReference(solution_interp[6:, val])
                    self.model.robot.move()

                    rospy.sleep(self.ik_dt)

                self.model.robot.sense()
                self.model.model.syncFrom(self.model.robot)


            # Second planning phase
            q_start = self.model.model.getJointPosition()
            self.q_bounder(q_start)
            self.model.ps.update()
            self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # compute a feasible pose with the lifted_contact 10 cm distant from the surface
            lifted_contact_final_pose.translation = np.array(self.stance_list[i+2][-1]['ref']['pose']) +\
                                                    scale * np.array(self.stance_list[i+2][-1]['ref']['normal'])

            self.ctrl_tasks[lifted_contact_ind].setPoseReference(lifted_contact_final_pose)
            self.ctrl_tasks[lifted_contact_ind].setPoseTarget(lifted_contact_final_pose, 1)
            self.postural.setReferencePosture(self.model.model.eigenToMap(self.q_list[i+2]))

            self.model.model.setJointPosition(self.q_list[i+2])
            self.model.model.update()
            # self.ik_solver.solve()
            ci_time = 0.0
            time_from_reaching = 0.
            UNABLE_TO_SOLVE_MAX = 5.
            q1 = np.empty(shape=[self.model.model.getJointNum(), 0])

            print 'starting cartesian trajectory ...'
            while self.ctrl_tasks[lifted_contact_ind].getTaskState() == pyci.State.Reaching:
                q1 = np.hstack((q, self.model.model.getJointPosition().reshape(self.model.model.getJointNum(), 1)))

                if not self.ci_solve_integrate(ci_time):
                    print('Unable to solve!!!')
                    unable_to_solve += 1
                    print(unable_to_solve)
                    # break
                    if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
                        print("Maximum number of unable_to_solve reached: ")
                        print(unable_to_solve)
                        return q1, False
                else:
                    unable_to_solve = 0

                ci_time += self.ik_dt
            print 'done!'

            self.model.model.setJointPosition(q1[:, -1])
            self.model.model.update()
            self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

            if not self.model.state_vc(self.model.model.getJointPosition()):
                # update contact list and rotation matrices through '/contact' topic
                active_ind = [ind['ind'] for ind in self.stance_list[i + 1]]
                active_links = [self.model.ctrl_points[j] for j in active_ind]

                normals = [j['ref']['normal'] for j in self.stance_list[i + 1]]

                contacts = SetContactFrames()
                contacts.action = SetContactFrames.SET
                contacts.frames_in_contact = active_links
                contacts.rotations = [eigenpy.Quaternion(self.rotation(elem)) for elem in normals]
                contacts.friction_coefficient = 0.5 * np.sqrt(2)

                self.contacts_pub.publish(contacts)

                if not self.NSPG.sample(60.0):
                    print('Error: Unable to find a collision free and stable pose!')
                    self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())
                    exit()

            np.savetxt('q_goal.csv', np.array(self.model.model.getJointPosition()), delimiter=',')
            q_goal = self.model.model.getJointPosition()
            self.q_bounder(q_goal)
            self.model.model.setJointPosition(q_goal)
            self.model.model.update()
            self.model.ps.update()
            self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # set all contacts to be active for second planning phase
            active_ind = [ind['ind'] for ind in self.stance_list[i+2]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  # names
            self.model.cs.setContactLinks(active_links)

            # set rotation matrix for each contact
            normals = [j['ref']['normal'] for j in self.stance_list[i+2]]
            [self.model.cs.setContactRotationMatrix(k, j) for k, j in zip(active_links,  [self.rotation(elem) for elem in normals])]

            solution = None
            print 'starting second planning phase ...'
            while solution is None and index < self.MAX_RRT_ATTEMPTS:
                index = index + 1

                if index == self.MAX_RRT_ATTEMPTS:
                    raise Exception("Unable to find a feasible plan!")

                solution, error = self.model.plan_step(q_start, q_goal, lifted_contact, planner_type='RRTConnect', timeout=60.0, threshold = 1e-3)
            solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.01)

            print 'done!'

            print 'starting interpolation (3/4) ...'
            for val in range(np.size(solution_interp, 1)):

                self.model.replay_model.setJointPosition(solution_interp[:, val])
                self.model.replay_model.update()
                self.model.model.setJointPosition(solution_interp[:, val])
                self.model.model.update()
                self.model.ps.update()
                self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())

                # send trajectory to robot
                self.model.robot.setPositionReference(solution_interp[6:, val])
                self.model.robot.move()


                rospy.sleep(self.ik_dt)

            print 'done!'

            exit()
            # Surface reacher
            self.surface_reacher(lifted_contact, i+2, 20)




