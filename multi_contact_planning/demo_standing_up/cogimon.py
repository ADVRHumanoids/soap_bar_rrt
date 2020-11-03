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


class Cogimon:

    def __init__(self, urdf, srdf, ctrl_points, logged_data):


        # make xbot model
        opt = co.ConfigOptions()
        opt.set_urdf(urdf)
        opt.set_srdf(srdf)
        opt.generate_jidmap()
        opt.set_bool_parameter('is_model_floating_base', True)
        opt.set_string_parameter('model_type', 'RBDL')
        self.model = xbot.ModelInterface(opt)
        # self.robot = xbot.RobotInterface(opt)
        self.replay_model = xbot.ModelInterface(opt)
        self.id_model = xbot.ModelInterface(opt)
        self.logged_data = logged_data

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

        # planning scene defines valid regions of the state space
        self.ps = validity_check.PlanningSceneWrapper(self.model)
        self.ps.startGetPlanningSceneServer()
        self.ps.startMonitor()

        # opensot uses linearized inner pyramid friction cone's approximation while cpl uses the non linear cone
        self.cs = validity_check.CentroidalStatics(self.model, self.ctrl_points.values(), 0.5*np.sqrt(2))

        # goal sampler
        # self.gs = GoalSampler(self.model, ctrl_points.values())

        # validity checker
        def is_model_state_valid():
            self.ps.update()
            self.rspub.publishTransforms('ci')

            # return not self.ps.checkCollisions() and self.cs.checkStability(5*1e-2)
            return not self.ps.checkCollisions()

        # set it to goal sampler
        # self.gs.set_validity_checker(is_model_state_valid)

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
            return is_model_state_valid()

        self.state_vc = is_state_valid

    def plan_step(self,
                  qstart, qgoal, swing_id=-1,
                  planner_type='RRTConnect', timeout=15.0, threshold = 0.01):


        # manifold

        constr = manifold.make_constraint(self.model, self.ctrl_points, swing_id)

        planner_config = {
            'state_space': {'type': 'Atlas'}
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

    def interpolate(self, solution, dt, s_threshold=0.01):

        qsize = solution.shape[0]
        nknots = solution.shape[1]

        seg_durs = []

        for i in range(nknots - 1):
            seg_durs.append(0.05)

        seg_durs.insert(0, 0)

        times = np.cumsum(seg_durs)

        interpolators = []
        for i in range(qsize):
            inter = interpol.UnivariateSpline(times, solution[i, :], s=s_threshold)
            interpolators.append(inter)

        nsamples = int(np.sum(seg_durs) / dt)
        solution_interp = np.zeros((qsize, nsamples))

        for i in range(len(interpolators)):
            for j in range(nsamples):
                solution_interp[i, j] = interpolators[i](dt * j)

        return solution_interp, [dt * j for j in range(nsamples)], times


