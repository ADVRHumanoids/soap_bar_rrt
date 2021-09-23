import yaml
from cartesian_interface.pyci_all import *
from cartesio_planning import constraints
import rospy

# class Manifold:
#     def __init__(self, model):
#         manifold_str = rospy.get_param("planner/problem_description_constraint")
#         manifold_dict = yaml.safe_load(manifold_str)
#
#         cs_str = yaml.dump(manifold_dict)
#         self.cs_ci = pyci.CartesianInterface.MakeInstance('OpenSot', cs_str, model, 0.1, '/tmp')
#
#     def as_constraint(self):
#         constr = constraints.CartesianConstraint(self.cs_ci)
#         return constr
#
#     def reset(self, dt):
#         self.cs_ci.reset(dt)


def make_constraint(model, ctrl_points, swing_id):

    # write cartesio config
    cs_cfg = dict()

    cs_cfg['solver_options'] = {'regularization': 1e-3, 'back_end': 'osqp'}

    if not swing_id == -1:
        cs_cfg['stack'] = [
            [c for c in ctrl_points.values() if c != ctrl_points[swing_id]]
        ]
    else:
        cs_cfg['stack'] = [
            [c for c in ctrl_points.values()]
        ]

    cs_cfg['constraints'] = ['joint_limits']

    cs_cfg['joint_limits'] = {
        'type': 'JointLimits',
    }

    for c in ctrl_points.values():
        if (c == "l_ball_tip" or c == "r_ball_tip"):
            cs_cfg[c] = {
                'type': 'Cartesian',
                'distal_link': c,
                'indices': [0, 1, 2],
                'lambda': .1
            }
        else:
            cs_cfg[c] = {
                'type': 'Cartesian',
                'distal_link': c,
                'lambda': .1
            }

    cs_str = yaml.dump(cs_cfg)
    cs_ci = pyci.CartesianInterface.MakeInstance('OpenSot', cs_str, model, 0.1, '/tmp/manifold')

    return constraints.CartesianConstraint(cs_ci)

def make_constraint_from_param_server(model):
    manifold_str = rospy.get_param("planner/problem_description_constraint")
    manifold_dict = yaml.safe_load(manifold_str)

    cs_str = yaml.dump(manifold_dict)
    cs_ci = pyci.CartesianInterface.MakeInstance('OpenSot', cs_str, model, 0.1, '/tmp')

    return constraints.CartesianConstraint(cs_ci)