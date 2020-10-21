from cartesio_planning.planning import PositionCartesianSolver
from xbot_interface.xbot_interface import ModelInterface
from cartesian_interface.pyci_all import *
import numpy as np
import scipy.linalg as la
import yaml
import time


class GoalSampler:

    def __init__(self, model, links):
        self.ik_str = self._generate_ik_cfg(links)
        self.ci = pyci.CartesianInterface.MakeInstance('OpenSot', self.ik_str, model, 1.)
        self.ik = PositionCartesianSolver(self.ci)
        self.model = model
        self.vc = lambda: True
        np.random.seed(120)

    def set_validity_checker(self, vc):
        self.vc = vc

    def set_desired_pose(self, link, pose):
        self.ik.setDesiredPose(link, pose)

    def sample_goal(self, timeout):

        qmin, qmax = self.model.getJointLimits()

        q = self.model.getJointPosition()
        goal_found = False

        tic = time.time()
        while not goal_found:

            self.model.setJointPosition(q)
            self.model.update()

            if not self.ik.solve():
                print('Not converging..')
            else:
                print('Converged!')
                goal_found = self.vc()

            if not goal_found:
                print('validity check returned false, retry!')
                N = self._nullspace()
                rho = 2

                delta_q = N.dot(np.random.uniform(-rho, rho, size=N.shape[1]))
                q = q + delta_q
                q = np.minimum(q, qmax)
                q = np.maximum(q, qmin)

            toc = time.time()
            elapsed = toc - tic
            if elapsed >= timeout:
                print('timeout')
                return None

        goal = self.model.getJointPosition()
        goal = np.minimum(goal, qmax)
        goal = np.maximum(goal, qmin)
        self.model.setJointPosition(goal)
        self.model.update()

        if not self.vc():
            print('Invalid state')
            return None

        if np.any(goal > qmax) or np.any(goal < qmin):
            print((goal > qmax) * (goal - qmax))
            print((goal < qmin) * (qmin - goal))
            print('Invalid bounds')
            return None

        return goal

    def refine_goal(self, goal, q0):

        ret = goal.copy()

        self.model.setJointPosition(q0)
        self.model.update()
        valid = self.vc()

        lam = 0.1

        while valid:
            step = lam * (q0 - goal)
            N = self._nullspace()
            step_ns = N.dot(N.transpose).dot(step)

            if la.norm(step_ns) < 0.01:
                break

            goal_trial = goal + step_ns

            self.model.setJointPosition(goal_trial)
            self.model.update()

            valid = self.ik.solve() and self.vc()

            if valid:
                ret = self.model.getJointPosition()

        return ret

    def _nullspace(self):
        J = self.ik.getJacobian()
        U, S, V = la.svd(J)
        null_dim = J.shape[1] - J.shape[0]
        N = V[:, -null_dim - 1:-1]  # last nulldim cols
        return N

    def _generate_ik_cfg(self, links):

        # write cartesio config
        gs_cfg = dict()

        gs_cfg['solver_options'] = {'regularization': 1e-3}

        gs_cfg['stack'] = [
            list(links), ["postural"]  # discard postural
        ]

        gs_cfg['constraints'] = ['velocity_limits', 'joint_limits']

        gs_cfg['joint_limits'] = {
            'type': 'JointLimits'
        }

        gs_cfg['velocity_limits'] = {
            'type': 'VelocityLimits',
            'limits': 0.5  # rads^-1
        }

        for c in links:
            gs_cfg[c] = {
                'type': 'Cartesian',
                'distal_link': c
            }

        gs_cfg['postural'] = {
            'type': 'Postural',
            'lambda': 0.
        }

        return yaml.dump(gs_cfg)
