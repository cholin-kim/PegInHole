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

    sys.path.append(os.path.abspath("/home/surglab/panda_teleassembly_2/lib_py")) # pandaKinematics에서 'import pandaVar'을 하기 때문에 경로를 추가해줘야함
    from pandaKinematics import pandaKinematics
    from object_data import object_data



    # create sim, publisher for sim
    sim = pandaRvizTeleassembly(is_core=False, use_GUI=False)

    env_sim = pandaRvizMotionTeleassembly(ns='environment')
    panda_lift = pandaRvizMotionTeleassembly(ns='panda_lift')

    marking = pandaRvizMarkerTeleassembly(ns='panda_marker')

    obj = object_data()

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
    arm_des2 = [0.0, -1, -0.5, -2, 0.3, 2, -0.3]
    arm_des3 = [0.0, -1, -0.5, -2, 0.3, 2, -0.3]

    # grip_des : q of fingers of gripper
    grip_des = [0, 0, 0]

    # time
    t = 0.0



    listener = tf.TransformListener()



    # Simulation
    while not rospy.is_shutdown():
        # rotation of lift on rail (free)
        ang = 0.0 + 1.5 * np.cos(0.15 * np.pi * t)
        r = -0.625
        pos_xyz = [r*np.cos(ang), r*np.sin(ang), 0.03]
        rot_euler = [0, 0, ang]

        # stroke of lift
        lift_des[0] = 0.5# + 0.24 * np.cos(0.12 * np.pi * t)

        # moving arm2
        # arm_des2[0] = 0.0# + 0.2 * np.cos(0.1 * np.pi * t)
        # arm_des2[1] = -1# + 0.5 * np.cos(0.2 * np.pi * t)
        # arm_des2[2] = 0.0# + 0.3 * np.cos(0.13 * np.pi * t)
        # arm_des2[3] = -2.5# + 0.5 * np.cos(0.17 * np.pi * t)
        # arm_des2[4] = 0# + 0.7 * np.cos(0.3 * np.pi * t)
        # arm_des2[5] = 2.5# + 0.7 * np.cos(0.15 * np.pi * t)
        # arm_des2[6] = 0.0# + 2.0 * np.cos(0.5 * np.pi * t)

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
        ang_obj = 0.0# + 1.0 * np.cos(0.3 * np.pi * t)

        # color of marker
        # color = [0.6 + 0.35 * np.cos(2.5 * np.pi * t), 0.6 + 0.35 * np.cos(2.5 * np.pi * (t+1.3)), 0.6 + 0.35 * np.cos(2.5 * np.pi * (t+2.7)), 0.6]


        # build joint msg to publish ( /panda_sim/panda_lift/joint_states )
        total_joint = lift_des + arm_des2 + grip_des + arm_des3

        # publish msg
        panda_lift.set_lift_tf(pos_xyz=pos_xyz, rot_euler=rot_euler)
        panda_lift.set_joint_position_direct(joints=total_joint)
        env_sim.set_joint_position_direct(joints=[ang_obj])


        # Find Nearst
        # ... tf를 얻으려면 tf가 있어야 하는데 tf가 있으려면 기존에 각도가 정의되어야 함... 그러니까 이거 받아오기 전에 각도를 한번 정도는 publish 해야함

        try:
            (trs_f, rot_f) = listener.lookupTransform('/base', '/panda2_link_finger_end', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trs_r, rot_r) = listener.lookupTransform('/base', '/panda2_link0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        try:
            (trs_o, rot_o) = listener.lookupTransform('/base', '/object', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        L_h_pos = obj.find_hole_pos(pos_obj=[trs_o, rot_o], L_hole=None)
        mtx_b2r = obj._get_mtx([trs_r, rot_r])



        L_L_q = []

        for i in range(0, 5):
            mtx_b2h = obj._get_mtx(L_h_pos[i][0])
            mtx_r2h = np.linalg.inv(mtx_b2r) @ mtx_b2h

            mtx_r2h[:3, 1] = -mtx_r2h[:3, 1]
            mtx_r2h[:3, 2] = -mtx_r2h[:3, 2]

            for j in range(1):
                angle = 2*np.pi / 8 * j
                mtx_rot = np.array([
                    [np.cos(angle), -np.sin(angle),   0,   0],
                    [np.sin(angle),  np.cos(angle),   0,   0],
                    [            0,              0,   1,   0],
                    [            0,              0,   0,   1],
                ])
                L_q = pandaKinematics.ik(mtx_r2h @ mtx_rot, q0=arm_des2, k=0.1)
                # print(f'i : {i} , j : {j} , L_q : {L_q}')

                if not (L_q is None):
                    L_L_q.append(L_q)
        
        # ik를 5번 하는 것도 오래 걸림... 시뮬레이션 기준으로는 너무 오래 걸림
        # 아니면... 어차피 원통 좌표계 처럼 움직이니까... 로봇 베이스에 따른 manipulability가 괜찮은 영역을 적당히 구한 다음에 그거 저장하고 구멍이 그 영역에 있는지 판단하는 방식으로?

        print(len(L_L_q))
        

        # arm_des2 = list(L_q.reshape(-1))




        
        # build joint msg to publish ( /panda_sim/panda_lift/joint_states )
        total_joint = lift_des + arm_des2 + grip_des + arm_des3

        # publish msg
        panda_lift.set_lift_tf(pos_xyz=pos_xyz, rot_euler=rot_euler)
        panda_lift.set_joint_position_direct(joints=total_joint)
        env_sim.set_joint_position_direct(joints=[ang_obj])




        # # Visualize manipulability (about posision xyz)
        # Tbs = pandaKinematics.fk(joints=arm_des2[:])[0]
        # J = pandaKinematics.jacobian(Tbs)

        # U, s, _ = np.linalg.svd(J[:3])
        # mtx_rot = U[:3, :3]
        # mtx_trs = np.array(trs_f)

        # marking.set_sphere(id=50, pos=mtx_trs, ori=Rotation.from_matrix(mtx_rot).as_quat(), scale=s)



        # set & publish marker (each hole)
        # L_pos = [ L_h_order[i][0][0] for i in range(len(L_h_order)) ]   # 똑같은 위치를 2번 계산하면 느리지 않을까 해서...
        # L_ori = [ L_h_order[i][0][1] for i in range(len(L_h_order)) ]

        # L_color = []
        # for i in range(0, 10):                      # 가장 가까운 10개는 청록색, 나머지는 빨간색으로 표시
        #     L_color.append([0, 1, 1, 0.8])
        # for i in range(10, len(L_h_order)):
        #     L_color.append([0.7, 0.2, 0.2, 0.7])

        # marking.set_cylinders(
        #     id_start=150,
        #     L_pos=L_pos,
        #     L_ori=L_ori,
        #     L_color=L_color
        # )



        t += 0.01
        time.sleep(0.01)
