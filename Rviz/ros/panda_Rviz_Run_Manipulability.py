import rospy
import numpy as np
import tf
from scipy.spatial.transform import Rotation





if __name__ == "__main__":
    import time
    import sys, os

    sys.path.append(os.path.abspath("/home/surglab"))   # appending address where panda_teleassembly directory is located
    from panda_teleassembly_2.ros.pandaRvizMotionTeleassembly import pandaRvizMotionTeleassembly
    from panda_teleassembly_2.ros.pandaRvizTeleassembly import pandaRvizTeleassembly
    from panda_teleassembly_2.ros.pandaRvizMarkerTeleassembly import pandaRvizMarkerTeleassembly

    sys.path.append(os.path.abspath("/home/surglab/panda_teleassembly_2/panda_py")) # pandaKinematics에서 'import pandaVar'을 하기 때문에 경로를 추가해줘야함
    from pandaKinematics import pandaKinematics



    # create sim, publisher for sim
    sim = pandaRvizTeleassembly(is_core=False, use_GUI=False)

    env_sim = pandaRvizMotionTeleassembly(ns='environment')
    panda_lift = pandaRvizMotionTeleassembly(ns='panda_lift')

    marking = pandaRvizMarkerTeleassembly(ns='panda_marker')

    """
    < topic name > : /panda_sim/panda_lift/joint_states
    < contents > : joint_states
        name: 
    - joint_lift_base       (length lifted by lifter)
    - panda2_joint1         (angle of robot joint... below things are similar)
    - panda2_joint2
    - panda2_joint3
    - panda2_joint4
    - panda2_joint5
    - panda2_joint6
    - panda2_joint7
    - panda2_joint_finger1
    - panda2_joint_finger2
    - panda2_joint_finger3
    - fr3_joint1
    - fr3_joint2
    - fr3_joint3
    - fr3_joint4
    - fr3_joint5
    - fr3_joint6
    - fr3_joint7
    """

    # lift_des = [ stroke of lift ]
    lift_des = [0]

    # arm_des2 : angles of arm2
    # arm_des3 : angles of arm3
    arm_des2 = [0.0330, 0.3160, 0.3840, 0.0880, 0.1070, 0.0, 0.0]
    arm_des3 = [0.6330, 0.3160, 0.3840, 0.0880, 0.1070, 0.0, 0.0]

    # grip_des : q of fingers of gripper
    grip_des = [0, 0, 0]

    # time
    t = 0.0



    listener = tf.TransformListener()



    # Simulation
    while not rospy.is_shutdown():
        # rotation of lift on rail (free)
        ang = 0.5 + 1.5 * np.cos(0.15 * np.pi * t)
        r = -0.625
        pos_xyz = [r*np.cos(ang), r*np.sin(ang), 0.03]
        rot_euler = [0, 0, ang]

        # stroke of lift
        lift_des[0] = 0.24 + 0.24 * np.cos(0.12 * np.pi * t)

        # moving arm2
        arm_des2[0] = 0.5 + 0.5 * np.cos(0.1 * np.pi * t)
        arm_des2[1] = -0.5 + 0.7 * np.cos(0.2 * np.pi * t)
        arm_des2[2] = 0.2 + 0.8 * np.cos(0.13 * np.pi * t)
        arm_des2[3] = -1.5 + 0.5 * np.cos(0.17 * np.pi * t)
        arm_des2[4] = 0 + 0.7 * np.cos(0.3 * np.pi * t)
        arm_des2[5] = 1.5 + 0.7 * np.cos(0.15 * np.pi * t)
        arm_des2[6] = 0.0 + 2.0 * np.cos(0.5 * np.pi * t)

        # moving arm3
        arm_des3[0] = -arm_des2[0]
        arm_des3[1] = arm_des2[1]
        arm_des3[2] = -arm_des2[2]
        arm_des3[3] = arm_des2[3]
        arm_des3[4] = -arm_des2[4]
        arm_des3[5] = arm_des2[5]
        arm_des3[6] = arm_des2[6]

        # moving gripper
        grip_des[0] = 0.01 + 0.01 * np.cos(1 * np.pi * t)
        grip_des[1] = grip_des[0]
        grip_des[2] = grip_des[0]

        # angle of object
        ang_obj = 0.0# + 1.0 * np.cos(0.3 * np.pi * t)

        # color of marker
        color = [0.6 + 0.35 * np.cos(2.5 * np.pi * t), 0.6 + 0.35 * np.cos(2.5 * np.pi * (t+1.3)), 0.6 + 0.35 * np.cos(2.5 * np.pi * (t+2.7)), 0.6]



        # build joint msg to publish ( /panda_sim/panda_lift/joint_states )
        total_joint = lift_des + arm_des2 + grip_des + arm_des3

        # publish msg
        panda_lift.set_lift_tf(pos_xyz=pos_xyz, rot_euler=rot_euler)
        panda_lift.set_joint_position_direct(joints=total_joint)
        env_sim.set_joint_position_direct(joints=[ang_obj])

        # set & publish marker (each hole)
        id_start = 100
        L_color = [color for i in range(len(marking.hole_name))]
        marking.set_cylinder_object(ang_obj=ang_obj, id_start=id_start, L_color=L_color)



        # # Visualize manipulability (about posision xyz)
        try:
            (trans, rot) = listener.lookupTransform('/base', '/panda2_link_finger_end', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        for k in range(1):
            Tbs = pandaKinematics.fk(joints=arm_des2[:])[0]
            J = pandaKinematics.jacobian(Tbs)
            U, s, _ = np.linalg.svd(J[:3])
            mtx_rot = U[:3, :3]
            mtx_trs = np.array(trans)
        marking.set_sphere(id=50, pos=mtx_trs, ori=Rotation.from_matrix(mtx_rot).as_quat(), scale=s)



        t += 0.01
        time.sleep(0.01)
