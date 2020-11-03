import cogimon
from cartesian_interface.pyci_all import *
import rospy
import yaml
import numpy as np
from cartesian_interface import pyest
import manifold

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

        ik_cfg['solver_options'] = {'regularization': 1e-4, 'back-end': 'osqp'}

        ik_cfg['stack'] = [
            self.model.ctrl_points.values(), ['com'], ['postural']
        ]

        ik_cfg['postural'] = {
            'name': 'postural',
            'type': 'Postural',
            'lambda': 0.01,
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
                    'indices': [0, 1, 2],
                    'lambda': .1
                }
            else:
                ik_cfg[c] = {
                    'type': 'Cartesian',
                    'distal_link': c,
                    'lambda': .1
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

    def impact_detector(self, task, magnitude):

        detect_bool = 0
        wrench = self.ft_map[task.getName()].getWrench()
        # wrench[direction] = 0 # FOR SIMULATION
        direction = [i for i, e in enumerate(self.stance_list[task.getName()]['ref']['normal']) if e != 0]
        if (wrench[direction] >= magnitude):
            detect_bool = 1

        return detect_bool

    def surface_reacher(self, task, force_treshold):

        print 'starting surface reacher...'
        # velocity desired
        vel_ref = 0.01
        vel_task = np.hstack((vel_ref * (- np.array(self.stance_list[task]['ref']['normal'])), [0,0,0]))

        task.enable()
        task.setControlMode(pyci.ControlType.Velocity)
        lambda_value = task.getLambda()
        task.setLambda(0)

        while not self.impact_detector(task, force_treshold):

            if not self.impact_detector(task, force_treshold):
                task.setVelocityReference(vel_task)

            self.robot.sense()
            self.model.syncFrom(self.robot)
            self.f_est.update()


        task.enable()
        task.setControlMode(pyci.ControlType.Position)
        task.setLambda(lambda_value)

        self.ci.update()

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
            self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

            self.model.model.setJointPosition(q_start)
            self.model.model.update()
            self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

            q_goal = self.q_list[i+1]
            self.q_bounder(q_goal)
            self.model.model.setJointPosition(q_goal)
            self.model.model.update()
            self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # set all contacts to be active for first planning phase
            active_ind = [ind['ind'] for ind in self.stance_list[i]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  #names
            self.model.cs.setContactLinks(active_links)

            # set rotation matrix for each contact
            normals = [j['ref']['normal'] for j in self.stance_list[i]]

            # print normals
            [self.model.cs.setContactRotationMatrix(k,j) for k, j in zip(active_links,  [self.rotation(elem) for elem in normals])]

            solution = None
            print 'start first planning phase ...'
            while solution is None and index < self.MAX_RRT_ATTEMPTS:
                index = index + 1

                if index == self.MAX_RRT_ATTEMPTS:
                    error("Unable to find a feasible plan!")

                solution, error = self.model.plan_step(q_start, q_goal, planner_type='RRTConnect', timeout=60.0)

            print 'done!'
            print 'start interpolation ...'
            solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.01)
            print 'done!'

            print 'replicating trajectory (1/4) ...'
            for val in range(np.size(solution_interp, 1)):
                self.model.replay_model.setJointPosition(solution_interp[:,val])
                self.model.replay_model.update()
                self.model.model.setJointPosition(solution_interp[:,val])
                self.model.model.update()
                self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())
                # self.model.model.setPositionReference(solution_interp)
                # self.model.model.move()
                rospy.sleep(self.ik_dt)
            print 'done!'

            # get active contacts from 'stance_list' to retrieve lifted contact
            # hypo: there must be a lifted contact
            active_list = []
            lifted_contact = [x for x in list(self.model.ctrl_points.keys()) if
                              x not in [j['ind'] for j in self.stance_list[i + 1]]][0]

            # self.model.model.setJointPosition(self.q_list[i + 1])
            # self.model.model.update()
            # self.model.rspub.publishTransforms('ci')
            # raw_input('')
            self.make_cartesian_interface()

            scale = 0.1
            lifted_contact_final_pose = self.ctrl_tasks[lifted_contact].getPoseReference()[0]
            lifted_contact_final_pose.translation = lifted_contact_final_pose.translation + scale * np.array(self.stance_list[i+2][lifted_contact]['ref']['normal'])
            self.ctrl_tasks[lifted_contact].setPoseTarget(lifted_contact_final_pose, 1.0)

            # Cartesian part
            ci_time = 0.0
            time_from_reaching = 0.
            UNABLE_TO_SOLVE_MAX = 5.
            q = np.empty(shape=[self.model.model.getJointNum(), 0])

            print 'starting cartesian trajectory ...'
            while self.ctrl_tasks[lifted_contact].getTaskState() == pyci.State.Reaching:
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
                self.model.replay_model.setJointPosition(q[:,j])
                self.model.replay_model.update()
                self.model.model.setJointPosition(q[:,j])
                self.model.model.update()
                self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())

            print 'done!'

            print 'pose before second planning check: ', self.model.state_vc(q[-1])

            # Second planning phase
            # TODO: planner also for contact lifting
            if self.model.state_vc(q[-1]):
                q_start = self.model.model.getJointPosition()
            else:
                q_start = self.q_list[i+1]
            self.q_bounder(q_start)
            self.model.start_viz.publishMarkers(self.model.ps.getCollidingLinks())

            q_goal = self.q_list[i+2]
            self.q_bounder(q_goal)
            self.model.model.setJointPosition(q_goal)
            self.model.model.update()
            self.model.goal_viz.publishMarkers(self.model.ps.getCollidingLinks())

            # set all contacts to be active for second planning phase
            active_ind = [ind['ind'] for ind in self.stance_list[i+1]]
            active_links = [self.model.ctrl_points[j] for j in active_ind]  #names
            self.model.cs.setContactLinks(active_links)

            # set rotation matrix for each contact
            normals = [j['ref']['normal'] for j in self.stance_list[i+1]]
            [self.model.cs.setContactRotationMatrix(k,j) for k, j in zip(active_links,  [self.rotation(elem) for elem in normals])]

            solution = None
            print 'starting second planning phase ...'
            while solution is None and index < self.MAX_RRT_ATTEMPTS:
                index = index + 1

                if index == self.MAX_RRT_ATTEMPTS:
                    raise Exception("Unable to find a feasble plan!")

                solution, error = self.model.plan_step(q_start, q_goal, lifted_contact, planner_type='RRTConnect', timeout=15.0)
            solution_interp, times, knot_times = self.model.interpolate(solution, self.ik_dt, s_threshold=0.01)

            print 'done!'

            print 'starting interpolation (3/4) ...'
            for val in range(np.size(solution_interp, 1)):

                self.model.replay_model.setJointPosition(solution_interp[:,val])
                self.model.replay_model.update()
                self.model.model.setJointPosition(solution_interp[:, val])
                self.model.model.update()
                self.model.sol_viz.publishMarkers(self.model.ps.getCollidingLinks())
                # self.model.model.setPositionReference(solution_interp)
                # self.model.model.move()
                rospy.sleep(self.ik_dt)

            print 'done!'
            # Surface reacher
            self.surface_reacher(self.ctrl_tasks[lifted_contact], 20)




