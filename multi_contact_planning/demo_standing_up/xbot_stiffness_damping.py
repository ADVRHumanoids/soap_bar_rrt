import rospy
import xbot_interface.xbot_interface as xbot

def setStiffnessAndDamping(robot, N_ITER, multiplier):

    K = robot.getStiffness()
    D = robot.getDamping()

    Kd = multiplier * K
    Dd = multiplier * D

    for k in range(N_ITER):
        stiff = list()
        damp = list()
        for K_start, K_end, D_start, D_end in zip(K, Kd, D, Dd):
            stiff.append(K_start + float(k) / (N_ITER - 1) * (K_end - K_start))
            damp.append(D_start + float(k) / (N_ITER - 1) * (D_end - D_start))
        robot.setStiffness(stiff)
        robot.setDamping(damp)

        # print "Completed: ", float(k) / N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    print "Stiffness of robot is: ", robot.getStiffness()
    print "Damping of robot is: ", robot.getDamping()
