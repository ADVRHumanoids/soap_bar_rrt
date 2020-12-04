from cartesio_planning import planning
from cartesio_planning import validity_check
from cartesio_planning import visual_tools
from cartesio_planning import constraints

import scipy.interpolate as interpol

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from cartesian_interface.pyci_all import *
import numpy as np
import yaml
import rospy
from goal_sampler import GoalSampler
import manifold

import cartesio_planning.NSPG
import cartesio_planning.validity_check

import q_connector



class Cogimon:

    def __init__(self, urdf, srdf, ctrl_points, logged_data):


        # make xbot model
        opt = co.ConfigOptions()
        opt.set_urdf(urdf)
        opt.set_srdf(srdf)
        opt.generate_jidmap()
        opt.set_bool_parameter('is_model_floating_base', True)
        opt.set_string_parameter('model_type', 'RBDL')
        opt.set_string_parameter('framework', 'ROS')
        self.model = xbot.ModelInterface(opt)
        # self.robot = xbot.RobotInterface(opt)
        # self.robot.setControlMode(xbot.ControlMode.Position())
        self.replay_model = xbot.ModelInterface(opt)
        self.id_model = xbot.ModelInterface(opt)
        self.logged_data = logged_data

        # update from robot
        # self.robot.sense()
        # self.model.syncFrom(self.robot)
        # self.model.update()

        # name of control frames
        self.ctrl_points = ctrl_points

        # visualization tools
        self.rspub = pyci.RobotStatePublisher(self.model)

        self.start_viz = visual_tools.RobotViz(self.model,
                                               '/cogimon/start',
                                               color=[0, 0, 1, 0.5],
                                               tf_prefix='ci/')

        self.goal_viz = visual_tools.RobotViz(self.model,
                                              '/cogimon/goal',
                                              color=[0, 1, 0, 0.5],
                                              tf_prefix='ci/')

        self.sol_viz = visual_tools.RobotViz(self.replay_model,
                                             '/cogimon/solution',
                                             color=[0.5, 0, 0.5, 0.5],
                                             tf_prefix='ci/')

        self.planner_viz = visual_tools.RobotViz(self.replay_model,
                                             '/cogimon/planner',
                                             color=[0., 0.5, 0.5, 0.5],
                                             tf_prefix='ci/')

        # planning scene defines valid regions of the state space
        self.ps = validity_check.PlanningSceneWrapper(self.model)
        self.ps.addBox("wall", [1.0, 3.0, 3.0], Affine3([1.6, 0.0, 1.5]))
        self.ps.addBox("ground", [3.0, 3.0, 1.0], Affine3([0.0, 0.0, -0.52]))
        self.ps.startGetPlanningSceneServer()
        self.ps.startMonitor()

        # opensot uses linearized inner pyramid friction cone's approximation while cpl uses the non linear cone
        self.cs = validity_check.CentroidalStatics(self.model,
                                                   self.ctrl_points.values(),
                                                   0.5*np.sqrt(2),
                                                   optimize_torque=False,
                                                   xlims_cop=np.array([-0.05, 0.1]),
                                                   ylims_cop=np.array([-0.05, 0.1]))

        # joint limits for the planner
        qmin, qmax = self.model.getJointLimits()

        qmin[0:6] = np.full(6, -6.0)
        qmax[0:6] = -qmin[0:6]

        qmin[0:3] = np.full(3, -6.)
        qmax[0:3] = -qmin[0:3]

        self.qmin = qmin
        self.qmax = qmax

        # state validity checker
        def is_state_valid(q):
            self.model.setJointPosition(q)
            self.model.update()
            return self.is_model_state_valid()

        self.state_vc = is_state_valid

        # publish robot
        self.rspub.publishTransforms('ci')

    # validity checker
    def is_model_state_valid(self):
        self.ps.update()
        self.rspub.publishTransforms('ci')

        in_collision = self.ps.checkCollisions()
        stable = self.cs.checkStability(5 * 1e-2)

        if in_collision:
            print "collision state!"
        if not stable:
            print "not stable state!"

        return not in_collision and stable

    def sensors_init(self, arm_estimation_flag):

        ft_map = self.robot.getForceTorque()

        if (arm_estimation_flag):
            # create force estimator
            indices_wrench = [0, 1, 2]
            ft_map['l_ball_tip'] = self.f_est.addLink('l_ball_tip', indices_wrench, ['left_arm'])
            ft_map['r_ball_tip'] = self.f_est.addLink('r_ball_tip', indices_wrench, ['right_arm'])
            self.f_est.update()

        return ft_map

    def plan_step(self,
                  qstart, qgoal, swing_id=-1,
                  planner_type='RRTConnect', timeout=15.0, threshold = 0.01):


        # manifold

        constr = manifold.make_constraint(self.model, self.ctrl_points, swing_id)

        planner_config = {
            'state_space': {'type': 'Atlas', 'alpha': np.pi/16, 'epsilon': 0.1, 'rho': 0.1}
        }

        # create planner
        planner = planning.OmplPlanner(
            constr,
            self.qmin, self.qmax,
            yaml.dump(planner_config)
        )

        planner.setStartAndGoalStates(qstart, qgoal, threshold)
        planner.setStateValidityPredicate(self.state_vc)
        success = planner.solve(timeout, planner_type)


        print('Planner output : {}'.format(success))

        if success:
            solution = np.array(planner.getSolutionPath()).transpose()
            error = solution[:, -1] - np.array(qgoal)
            return solution, error
        else:
            return None, None

    def play_on_rviz(self, solution, ntimes, duration, viz, model):

        # play solution a number of times..
        dt = duration / solution.shape[1]

        for _ in range(ntimes):

            for i in range(solution.shape[1]):
                q = solution[:, i]
                model.setJointPosition(q)
                model.update()
                viz.publishMarkers()
                rospy.sleep(dt)

    def interpolate(self, solution, dt, s_threshold=0.):

        qsize = solution.shape[0]
        nknots = solution.shape[1]

        seg_durs = []

        for i in range(nknots - 1):
            seg_durs.append(0.05)

        seg_durs.insert(0, 0)

        times = np.cumsum(seg_durs)

        interpolators = []
        for i in range(qsize):
            if nknots < 4:
                inter = interpol.UnivariateSpline(times, solution[i, :], k = nknots - 1, s=s_threshold)
            else:
                inter = interpol.UnivariateSpline(times, solution[i, :], s=s_threshold)
            interpolators.append(inter)

        nsamples = int(np.sum(seg_durs) / dt)
        solution_interp = np.zeros((qsize, nsamples))

        for i in range(len(interpolators)):
            for j in range(nsamples):
                solution_interp[i, j] = interpolators[i](dt * j)

        return solution_interp, [dt * j for j in range(nsamples)], times

