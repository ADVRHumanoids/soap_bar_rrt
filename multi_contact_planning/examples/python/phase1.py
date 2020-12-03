#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from cartesio_planning.srv import CartesioPlanner

class planner_client(object):
    def __init__(self):
        self.__goal_pub = rospy.Publisher('planner/goal/joint_states', JointState, queue_size=10, latch=True)

    def publishGoal(self, joint_names, goal):
        goal_msg = self.__createGoalMsg(joint_names, goal)
        self.__goal_pub.publish(goal_msg)

    def solve(self, PLAN_MAX_ATTEMPTS, planner_type, plan_time, interpolation_time, goal_threshold):
        rospy.wait_for_service('planner/compute_plan')
        PLAN_ATTEMPTS = 0
        plan_success = False

        while PLAN_ATTEMPTS < PLAN_MAX_ATTEMPTS:
            try:

                plan = rospy.ServiceProxy('planner/compute_plan', CartesioPlanner)
                response = plan(planner_type=planner_type, time=(plan_time * (PLAN_ATTEMPTS + 1)), interpolation_time=interpolation_time,
                                goal_threshold=goal_threshold)
                if response.status.val == 6:  # EXACT_SOLUTION
                    print("EXACT_SOLUTION FOUND")
                    return True
                elif response.status.val == 5:  # APPROXIMATE SOLUTION
                    print("APPROXIMATE_SOLUTION FOUND")
                    PLAN_ATTEMPTS += 1
                else:
                    rospy.logerr("PLANNER RETURNED ERROR: %i", response.status.val)
                    PLAN_ATTEMPTS += 1

            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return False

        rospy.logerr("PLANNER CAN NOT FIND A SOLUTION, EXITING")
        return False

    ##PRIVATE
    def __createGoalMsg(self, joint_names, goal_config):
        """
        To create start and goal msgs from start and goal configs lists
        """
        goal_pose = JointState()

        for i in range(len(joint_names)):
            goal_pose.name.append(joint_names[i])
            goal_pose.position.append(goal_config[i])

        return goal_pose



if __name__ == '__main__':
    joint_names = ["VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5", "VIRTUALJOINT_6",
                   "LHipLat", "LHipSag", "LHipYaw", "LKneePitch", "LAnklePitch", "LAnkleRoll",
                   "RHipLat", "RHipSag", "RHipYaw", "RKneePitch", "RAnklePitch", "RAnkleRoll",
                   "WaistLat", "WaistYaw",
                   "LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2",
                   "RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"]
    goal_pose = [-0.15368987081904212, -0.10314795366518807, 0.7640134656474494, 3.4844940641908996e-05, 1.003581914293611, -3.785843110696436e-05, -3.953783628909849e-05, -1.9198621771899997, -2.1363360846215855e-05, 0.7922061790512522, 0.12407408333227256, 1.3795065767494878e-06, -3.9537658402357405e-05, -1.9198621771899997, -2.136311438261588e-05, 0.7921945501430171, 0.12408571224052402, 1.3796065471541717e-06, -1.8660286220993518e-05, 8.489006516230123e-06, -0.19445292297202182, 0.46857182904733846, 0.14653695168167274, -1.2247166922656882, 0.027710856918766916, -0.26325518320383146, 3.7626266681327204e-05, -0.1944428210179786, -0.46859844701659054, -0.1465736712234979, -1.2247131748749565, -0.027654977402383155, -0.2632529596729626, 4.140472081021349e-05]

    rospy.init_node('phase1', anonymous=True)

    planner_client = planner_client()

    for i in range(5):
        planner_client.publishGoal(joint_names, goal_pose)
        rospy.sleep(1.)

    if not planner_client.solve(PLAN_MAX_ATTEMPTS=5, planner_type="RRTstar", plan_time=30, interpolation_time=0.01, goal_threshold=0.01):
        print("ERROR! PLANNER CAN NOT FIND A SOLUTION!")
