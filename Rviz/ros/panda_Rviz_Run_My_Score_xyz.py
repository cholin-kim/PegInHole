import rospy
import numpy as np
import tf
from scipy.spatial.transform import Rotation





if __name__ == "__main__":
    import time
    import sys, os

    from Rviz.ros.pandaRvizMotionTeleassembly import pandaRvizMotionTeleassembly
    from Rviz.ros.pandaRvizTeleassembly import pandaRvizTeleassembly

    # create sim, publisher for sim
    sim = pandaRvizTeleassembly(is_core=False, use_GUI=False)

    env_sim = pandaRvizMotionTeleassembly(ns='environment')
    panda_lift = pandaRvizMotionTeleassembly(ns='panda_lift')


    # lift_des = [ stroke of lift ]
    lift_des = [0]

    # arm_des2 : angles of arm2
    # arm_des3 : angles of arm3
    arm_des2 = [0.0, -1, -0.5, -2, 0.3, 2, -0.3]
    arm_des3 = [0.0, -1, 0.5, -2, -0.3, 2, 0.3]

    # grip_des : q of fingers of gripper
    grip_des = [0, 0, 0]

    # time
    t = 0.0



    # Simulation
    while not rospy.is_shutdown():
        # rotation of lift on rail (free)
        ang = 0.0 + 1.5 * np.cos(0.15 * np.pi * t)
        r = -0.625
        pos_xyz = [r*np.cos(ang), r*np.sin(ang), 0.03]
        rot_euler = [0, 0, ang]

        # stroke of lift
        lift_des[0] = 0.25 + 0.25 * np.cos(0.12 * np.pi * t)

        # moving arm2
        arm_des2[0] = 0.0   + 0.2 * np.cos(0.1 * np.pi * t)
        arm_des2[1] = -1    + 0.5 * np.cos(0.2 * np.pi * t)
        arm_des2[2] = 0.0   + 0.3 * np.cos(0.13 * np.pi * t)
        arm_des2[3] = -2.5  + 0.5 * np.cos(0.17 * np.pi * t)
        arm_des2[4] = 0     + 0.7 * np.cos(0.3 * np.pi * t)
        arm_des2[5] = 2.5   + 0.7 * np.cos(0.15 * np.pi * t)
        arm_des2[6] = 0.0   + 2.0 * np.cos(0.5 * np.pi * t)

        # moving arm3
        # arm_des3[0] = -arm_des2[0]
        # arm_des3[1] = arm_des2[1]
        # arm_des3[2] = -arm_des2[2]
        # arm_des3[3] = arm_des2[3]
        # arm_des3[4] = -arm_des2[4]
        # arm_des3[5] = arm_des2[5]
        # arm_des3[6] = arm_des2[6]

        # moving gripper
        grip_des[0] = 0.01 + 0.01 * np.cos(1 * np.pi * t)
        grip_des[1] = grip_des[0]
        grip_des[2] = grip_des[0]

        # angle of object
        ang_obj = 0.3926990817# + 1.0 * np.cos(0.3 * np.pi * t)



        # build joint msg to publish ( /panda_sim/panda_lift/joint_states ) ... to use tf after this code (preventing tf error)
        total_joint = lift_des + arm_des2 + grip_des + arm_des3

        # publish msg
        panda_lift.set_lift_tf(pos_xyz=pos_xyz, rot_euler=rot_euler)
        panda_lift.set_joint_position_direct(joints=total_joint)
        env_sim.set_joint_position_direct(joints=[ang_obj])



        # build joint msg to publish ( /panda_sim/panda_lift/joint_states )
        total_joint = lift_des + arm_des2 + grip_des + arm_des3

        # publish msg
        panda_lift.set_lift_tf(pos_xyz=pos_xyz, rot_euler=rot_euler)
        panda_lift.set_joint_position_direct(joints=total_joint)
        env_sim.set_joint_position_direct(joints=[ang_obj])


        t += 0.01
        time.sleep(0.01)
