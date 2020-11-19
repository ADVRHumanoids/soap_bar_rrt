import yaml
from cartesian_interface.pyci_all import *
from cartesio_planning import constraints


def make_constraint(model, ctrl_points, swing_id):

    # write cartesio config
    cs_cfg = dict()

    cs_cfg['solver_options'] = {'regularization': 1e-2, 'back_end': 'qpoases'}

    if not swing_id == -1:
        cs_cfg['stack'] = [
            [c for c in ctrl_points.values() if c != ctrl_points[swing_id]]#, ['postural']  # discard postural
        ]
    else:
        cs_cfg['stack'] = [
            [c for c in ctrl_points.values()]#, ['postural']  # discard postural
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
    cs_cfg['postural'] = {
        'name': 'postural',
        'type': 'Postural',
        'lambda': 0.1,
    }

    cs_str = yaml.dump(cs_cfg)
    cs_ci = pyci.CartesianInterface.MakeInstance('OpenSot', cs_str, model, 0.1, '/tmp/manifold')

    return constraints.CartesianConstraint(cs_ci)