#!/usr/bin/env python
import xbot_interface.config_options as xbot_opt
import rospy
from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
import numpy as np
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import yaml

class CartesianInterfaceSolver:

    def __init__(self, model, ik_dt, ctrl_points):

        # Cartesian task to lift the swing contact
        self.model = model
        self.ik_dt = ik_dt
        self.ctrl_points = ctrl_points

        self.rspub = pyci.RobotStatePublisher(self.model)

        self.ik_pb = self.__make_problem_desc_ik()
        self.log_path = '/tmp'
        self.ci = pyci.CartesianInterface.MakeInstance('OpenSot',
                                                  self.ik_pb,
                                                  self.model,
                                                  self.ik_dt,
                                                  log_path=self.log_path)

        self.ctrl_tasks = []
        for k in self.ctrl_points.values():
            self.ctrl_tasks.append(self.ci.getTask(k))

        self.com = self.ci.getTask('com')
        self.postural = self.ci.getTask('postural')

    def __make_problem_desc_ik(self):

        # write cartesio config
        ik_cfg = dict()

        ik_cfg['solver_options'] = {'regularization': 1e-4, 'back-end': 'osqp'}

        ik_cfg['stack'] = [
            self.ctrl_points.values(), ['com'], ['postural']
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

        for c in self.ctrl_points.values():
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

        q = self.model.getJointPosition()
        qdot = self.model.getJointVelocity()
        qddot = self.model.getJointAcceleration()

        q += qdot * self.ik_dt + 0.5 * qddot * self.ik_dt ** 2
        qdot += qddot * self.ik_dt

        self.model.setJointPosition(q)
        self.model.setJointVelocity(qdot)
        self.model.update()

        return True

    def reach(self, task, goal, duration):

        ## Cartesian part
        q = np.empty(shape=[self.model.getJointNum(), 0])

        task.setActivationState(pyci.ActivationState.Enabled)

        time_from_reaching = 0.
        unable_to_solve = 0
        ci_time = 0.0
        initialize_trj = False

        CONVERGENCE_TIME = 5.
        UNABLE_TO_SOLVE_MAX = 5

        task.setPoseTarget(goal, duration)

        while task.getTaskState() == pyci.State.Reaching or time_from_reaching <= CONVERGENCE_TIME:

            q = np.hstack((q, self.model.getJointPosition().reshape(self.model.getJointNum(), 1)))

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

            if task.getTaskState() == pyci.State.Online:
                if not initialize_trj:
                    initialize_trj = True

                time_from_reaching += self.ik_dt

            self.rspub.publishTransforms('ci')

    def __getitem__(self, idx):
        return self.ctrl_tasks[idx]

if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)

    rospy.init_node('ci_solver_try')
    roscpp_init('ci_solver_try', [])

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
    model = xbot.ModelInterface(opt)
    robot = xbot.RobotInterface(opt)

    robot.sense()
    model.syncFrom(robot)
    model.update()

    ctrl_points = {0: 'l_ball_tip', 1: 'r_ball_tip', 4: 'l_sole', 5: 'r_sole'}
    ci_solver = CartesianInterfaceSolver(model=model, ik_dt=0.01, ctrl_points=ctrl_points)
    print 'created cartesian interface'

    one_task = ci_solver[0]

    starting_pose = Affine3(pos=[model.getPose(one_task.getName()).translation[0],
                                 model.getPose(one_task.getName()).translation[1],
                                 model.getPose(one_task.getName()).translation[2]])

    starting_pose.linear = model.getPose(one_task.getName()).linear

    goal = Affine3(pos=[model.getPose(one_task.getName()).translation[0],
                        model.getPose(one_task.getName()).translation[1],
                        model.getPose(one_task.getName()).translation[2] + 0.5])

    goal.linear = model.getPose(one_task.getName()).linear

    ci_solver.reach(ci_solver[0], goal, 10.)
    ci_solver.reach(ci_solver[0], starting_pose, 10.)




