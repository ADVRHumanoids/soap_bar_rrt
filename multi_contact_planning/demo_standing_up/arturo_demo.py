import numpy as np
import rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import xbot_interface.config_options as xbot_opt

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from cartesian_interface.pyci_all import *
import numpy as np
import yaml
import rospy
from goal_sampler import GoalSampler
import manifold
from cartesio_planning import validity_check

def rotation(normal):

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

if __name__ == '__main__':
    np.set_printoptions(precision=3, suppress=True)

    rospy.init_node('standing_up')
    roscpp_init('standing_up', [])
    # define contacts for the ForcePublisher
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('robot_description')
    srdf = rospy.get_param('robot_description_semantic')

    logged_data = []
    log_path = '/tmp'
    ctrl_points = {0:'l_ball_tip', 1:'r_ball_tip', 4:'l_sole', 5:'r_sole'}

    opt = co.ConfigOptions()
    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('model_type', 'RBDL')
    model = xbot.ModelInterface(opt)

    cs = validity_check.CentroidalStatics(model, ctrl_points.values(), 0.5)

    # centroidal statics
    active_links = ['l_ball_tip', 'r_ball_tip', 'l_sole', 'r_sole']
    cs.setContactLinks(active_links)
    normals = [[0., 0., 1.], [0., 0., 1.], [0., 0., 1.], [0., 0., 1.]]
    [cs.setContactRotationMatrix(k, j) for k, j in zip(active_links, [rotation(elem) for elem in normals])]
    q = [0.241653, -0.100305, 0.52693, 0.000414784, 1.42905, -0.000395218, -0.00387022, -0.556391, -0.00594669, 0, -0.872665, 0.00508346, 0.00454263, -0.556387, 0.00702034,1.38778e-17, -0.872665, -0.00604698, 0.0221668, -0.0242965, 0.426473, 0.855699, 0.878297, -1.4623, 0.0958207, -0.208411,1.05876e-05, 0.255248, -0.850543, -0.792886, -1.47237,-0.0789541, -0.195656,1.75265e-05]
    model.setJointPosition(q)
    model.update()

    forces_list = [[49.1115, -35.6032, 223.419, 0, 0, 0], [53.278, 37.0859, 213.826, 0, 0, 0], [-31.8832, 32.0597, 92.9953, 0, 0, 0], [70.5062, -33.5424, 156.46, 0, 0, 0]]
    forces = dict(zip(active_links, np.array(forces_list)))
    cs.setForces(forces)

    print cs.checkStability()