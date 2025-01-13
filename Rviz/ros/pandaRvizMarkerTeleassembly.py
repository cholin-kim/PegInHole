import rospy
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import time
import os
# from panda.utils.Trajectory import Trajectory
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
np.set_printoptions(precision=5, suppress=True)
path = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]



class pandaRvizMarkerTeleassembly:
    def __init__(self, ns='panda_lift', q0=None):

        self.ns = ns

        # ROS subscriber
        # rospy.Subscriber(...)

        # ROS publisher
        self._set_marker_pub = rospy.Publisher('/panda_sim/' + self.ns + '/marker', Marker, queue_size=1)
        self._set_marker_arr_pub = rospy.Publisher('/panda_sim/' + self.ns + '/marker_arr', MarkerArray, queue_size=1)


        # variable
        self.dict_hole = {
            'hole_1_1' : [[3.14159265, -0.7849178, 1.57079633],     [0, 0.15004805897, 0.05]],
            'hole_1_2' : [[3.14159265, -0.7849178, 0],              [0.15004805897, 0, 0.05]],
            'hole_1_3' : [[-3.14159265, -0.7849178, -1.57079633],   [0, -0.15004805897, 0.05]],
            'hole_1_4' : [[-3.14159265, -0.7849178, -3.14159265],   [-0.15004805897, 0, 0.05]],

            'hole_2_1' : [[-1.57079, 0, 0],                         [0, 0.190, 0.11]],
            'hole_2_2' : [[-1.57079, 0, -0.78539],                  [0.13435, 0.13435, 0.11]],
            'hole_2_3' : [[-1.57079, 0, -1.57079],                  [0.190, 0, 0.11]],
            'hole_2_4' : [[-1.57079, 0, -2.35619],                  [0.13435, -0.13435, 0.11]],
            'hole_2_5' : [[-1.57079, 0, 3.14159],                   [0, -0.190, 0.11]],
            'hole_2_6' : [[-1.57079, 0, 2.35619],                   [-0.13435, -0.13435, 0.11]],
            'hole_2_7' : [[-1.57079, 0, 1.57079],                   [-0.190, 0, 0.11]],
            'hole_2_8' : [[-1.57079, 0, 0.78539],                   [-0.13435, 0.13435, 0.11]],

            'hole_3_1' : [[0, -1.1079824, -1.57079633],             [0, 0.1557, 0.2161]],
            'hole_3_2' : [[0, -1.1079824,  -2.35619449],            [0.11009652583, 0.11009652583, 0.2161]],
            'hole_3_3' : [[0, -1.1079824, -3.14159265],             [0.1557, 0, 0.2161]],
            'hole_3_4' : [[0, -1.1079824, 2.35619449],              [0.11009652583, -0.11009652583, 0.2161]],
            'hole_3_5' : [[0, -1.1079824, 1.57079633],              [0, -0.1557, 0.2161]],
            'hole_3_6' : [[0, -1.1079824, 0.78539816],              [-0.11009652583, -0.11009652583, 0.2161]],
            'hole_3_7' : [[0, -1.1079824, 0],                       [-0.1557, 0, 0.2161]],
            'hole_3_8' : [[0, -1.1079824, -0.78539816],             [-0.11009652583, 0.11009652583, 0.2161]],

            'hole_4_1' : [[0, -1.1079824, -1.57079633],             [0, 0.1078, 0.3121]],
            'hole_4_2' : [[0, -1.1079824, -2.35619449],             [0.07622611101, 0.07622611101, 0.3121]],
            'hole_4_3' : [[0, -1.1079824, -3.14159265],             [0.1078, 0, 0.3121]],
            'hole_4_4' : [[0, -1.1079824, 2.35619449],              [0.07622611101, -0.07622611101, 0.3121]],
            'hole_4_5' : [[0, -1.1079824, 1.57079633],              [0, -0.1078, 0.3121]],
            'hole_4_6' : [[0, -1.1079824, 0.78539816],              [-0.07622611101, -0.07622611101, 0.3121]],
            'hole_4_7' : [[0, -1.1079824, 0],                       [-0.1078, 0, 0.3121]],
            'hole_4_8' : [[0, -1.1079824, -0.78539816],             [-0.07622611101, 0.07622611101, 0.3121]],

            'hole_top' : [[0.0, 0.0, 0.0],                          [0.00000000e+00,  0.00000000e+00,  4.15000000e-01]],
        }

        self.hole_name = list(self.dict_hole.keys())


        # Initialize ROS node
        if not rospy.get_node_uri():
            rospy.init_node(self.ns + "_rviz_marker", anonymous=True)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')


    """
    Set functions
    """

    def set_points(self, id, L_marker, color=[1, 1, 0, 1], scale=[0.03, 0.03, 0.03], frame_id='base'):
        # 이거 왜 안되나 했더니 RVIZ에서 topic으로 marker를 따로 추가해야 하더라고...
        # 근데 왜 vs에서 한글이 안쳐지냐? ... 공식 홈페이지 걸로 다시 다운
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id    # rviz 좌측 global fixed frame의 이름
        msg.ns = 'marker_' + self.ns
        msg.id = id

        # marker type
        msg.type = Marker.POINTS
        msg.action = Marker.ADD
        # msg.lifetime = 0

        # color
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = color[0], color[1], color[2], color[3]

        # size of marker
        msg.scale.x, msg.scale.y, msg.scale.z = scale[0], scale[1], scale[2]

        # position of markers (# of markers == len(L_marker))
        for p in L_marker:
            msg.points.append(Point(p[0], p[1], p[2]))

        self._set_marker_pub.publish(msg)
    


    def set_sphere(self, id, pos, ori, color=[1, 1, 0, 0.5], scale=[0.015, 0.015, 0.025], frame_id='base'):
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id    # rviz 좌측 global fixed frame의 이름
        msg.ns = 'marker_' + self.ns
        msg.id = id

        # marker type
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        # msg.lifetime = 1 # 왜인지 모르겠지만 오류가 뜸

        # color
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = color[0], color[1], color[2], color[3]

        # size of marker
        msg.scale.x, msg.scale.y, msg.scale.z = scale[0], scale[1], scale[2]

        # position of markers
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos[0], pos[1], pos[2]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = ori[0], ori[1], ori[2], ori[3]

        self._set_marker_pub.publish(msg)



    def set_cylinder(self, id, pos, ori, color=[1, 1, 0, 0.5], scale=[0.015, 0.015, 0.025], frame_id='base'):
        msg = Marker()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id    # rviz 좌측 global fixed frame의 이름
        msg.ns = 'marker_' + self.ns
        msg.id = id

        # marker type
        msg.type = Marker.CYLINDER
        msg.action = Marker.ADD
        # msg.lifetime = 1 # 왜인지 모르겠지만 오류가 뜸

        # color
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = color[0], color[1], color[2], color[3]

        # size of marker
        msg.scale.x, msg.scale.y, msg.scale.z = scale[0], scale[1], scale[2]

        # position of markers
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos[0], pos[1], pos[2]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = ori[0], ori[1], ori[2], ori[3]

        self._set_marker_pub.publish(msg)
    


    def set_cylinders(self, id_start, L_pos, L_ori, L_color=None, L_scale=None, frame_id='base'):
        arr_msg = MarkerArray()

        color = []
        scale = []

        for ind in range(len(L_pos)):
            msg = Marker()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id    # rviz 좌측 global fixed frame의 이름
            msg.ns = 'marker_arr_' + self.ns
            msg.id = id_start + ind

            # marker type
            msg.type = Marker.CYLINDER
            msg.action = Marker.ADD
            # msg.lifetime = 1 # 왜인지 모르겠지만 오류가 뜸

            # color
            if L_color is None:
                color = [1, 1, 0, 1]
            else:
                color = L_color[ind]
            msg.color.r, msg.color.g, msg.color.b, msg.color.a = color[0], color[1], color[2], color[3]

            # size of marker
            if L_scale is None:
                scale = [0.01357, 0.01357, 0.015] # 옆면 구멍 -> 0.008 근데 top 부분은 0.01201로 좀 더 큼
            else:
                scale = L_scale[ind]
            msg.scale.x, msg.scale.y, msg.scale.z = scale[0], scale[1], scale[2]

            # position of markers
            pos = L_pos[ind]
            ori = L_ori[ind]
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos[0], pos[1], pos[2]
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = ori[0], ori[1], ori[2], ori[3]

            # print(pos)

            arr_msg.markers.append(msg)

        self._set_marker_arr_pub.publish(arr_msg)
    


    def set_cylinder_object(self, ang_obj=0, height_obj=1.055, id_start=0, L_color=None, L_hole=None, L_scale=None, frame_id='base'):
        L_pos = []
        L_ori = []

        if L_hole is None:
            L_hole = self.hole_name


        for i in range(len(L_hole)):
            data_hole = self.dict_hole[L_hole[i]]


            # # 되기는 한데 좀 많이 느린 것 같음
            # for k in range(100):
            # mtx_g2o = np.zeros((4, 4))
            # mtx_g2o[:3, :3] = Rotation.from_euler('xyz', [0, 0, ang_obj]).as_matrix()
            # mtx_g2o[:3, 3] = np.array([0, 0, height_obj])
            # mtx_g2o[3][3] = 1

            # mtx_o2h = np.zeros((4, 4))
            # mtx_o2h[:3, :3] = Rotation.from_euler('xyz', data_hole[0]).as_matrix()
            # mtx_o2h[:3, 3] = np.array(data_hole[1])
            # mtx_o2h[3][3] = 1

            # mtx_result = mtx_g2o @ mtx_o2h

            # pos = mtx_result[:3, 3]
            # ori = Rotation.from_matrix(mtx_result[:3, :3]).as_quat()


            # # 좀 더 빠른걸로 하드코딩 해서 보정... object회전시키면 좀 더 버벅이네...
            # for k in range(100):# 속도 비교하려고 일부러 느리게 하려고... 이거 해보면 확실히 하드코딩 해서 보정한게 조금 더 빠르긴 함... 마음에는 안들지만...
            pos = data_hole[1].copy()
            pos[2] += height_obj
            temp = pos.copy()
            sin_bojung = np.sin(ang_obj)
            cos_bojung = np.cos(ang_obj)
            pos[0] = temp[0]*cos_bojung - temp[1]*sin_bojung
            pos[1] = temp[0]*sin_bojung + temp[1]*cos_bojung

            temp = data_hole[0].copy()
            temp[2] += ang_obj
            ori = Rotation.from_euler('xyz', temp).as_quat()


            L_pos.append(pos)
            L_ori.append(ori)


        self.set_cylinders(id_start=id_start, L_pos=L_pos, L_ori=L_ori, L_color=L_color, L_scale=L_scale, frame_id=frame_id)










if __name__ == "__main__":
    import time
    import sys, os
    sys.path.append(os.path.abspath("/home/surglab"))   # appending address where panda_teleassembly directory is located
    # from
    # from ros.pandaRvizMotionTeleassembly import pandaRvizMotionTeleassembly
    from panda_teleassembly.ros.pandaRvizMotionTeleassembly import pandaRvizMotionTeleassembly
    from panda_teleassembly.ros.pandaRvizTeleassembly import pandaRvizTeleassembly
    # from panda.ros.pandaRvizMotion import pandaRvizMotion
    sim = pandaRvizTeleassembly(is_core=False, use_GUI=False)

    env_sim = pandaRvizMotionTeleassembly(ns='environment')

    panda_lift = pandaRvizMotionTeleassembly(ns='panda_lift')   # pandaRvizMotionTeleassembly는 그냥 publish 하는 파일이라서 만든거에 따라 publish 이름만 좀 바꾸면 됨... 근데 아직 안고침
    """
    < topic name > : /panda_sim/panda_lift/joint_states

    < contents > : joint_states
        name: 
    - joint_base            (angle between current and initial vector of lift... vector: centro of rail -> pos of lift on rail)
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
    """                     # 근데 이렇게 하는게 불편하다 싶으면 그냥 기존에 있던거 2개로 복사한 다음 각 lift 위치만 동일하게 싱크 맞춰도 상관 없을 것 같습니다

    marking = pandaRvizMarkerTeleassembly(ns='panda_marker')


    # lift_des = [ angle of lift , stroke of lift ]
    lift_des = [0, 0]

    # arm_des2 : angles of arm2
    # arm_des3 : angles of arm3
    arm_des2 = [0.0330, 0.3160, 0.3840, 0.0880, 0.1070, 0.0, 0.0]
    arm_des3 = [0.6330, 0.3160, 0.3840, 0.0880, 0.1070, 0.0, 0.0]

    # grip_des : q of fingers of gripper
    grip_des = [0, 0, 0]

    t = 0.0



    while not rospy.is_shutdown():
        # # moving lift
        lift_des[0] = 0.0 + np.pi/1.5 * np.cos(0.1 * np.pi * t)
        lift_des[1] = 0.24 + 0.24 * np.cos(0.12 * np.pi * t)

        # moving arm2
        arm_des2[0] = 0.0 + 0.5 * np.cos(0.1 * np.pi * t)
        arm_des2[1] = 0.5 + 0.7 * np.cos(0.2 * np.pi * t)
        arm_des2[2] = 0.0 + 0.8 * np.cos(0.13 * np.pi * t)
        arm_des2[3] = -1.5 + 0.5 * np.cos(0.17 * np.pi * t)
        arm_des2[4] = 0.0 + 0.7 * np.cos(0.3 * np.pi * t)
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



        # angle, color of object
        ang_obj = 0.0# + 1.0 * np.cos(0.3 * np.pi * t)
        color = [0.6 + 0.35 * np.cos(2.5 * np.pi * t), 0.6 + 0.35 * np.cos(2.5 * np.pi * (t+1.3)), 0.6 + 0.35 * np.cos(2.5 * np.pi * (t+2.7)), 0.6]



        # build TF msg to publish ( /panda_sim/panda_lift/joint_states )
        total_joint = lift_des + arm_des2 + grip_des + arm_des3

        # publish msg
        panda_lift.set_joint_position_direct(joints=total_joint)
        env_sim.set_joint_position_direct(joints=[ang_obj])



        # set & publish marker (each hole)
        id_start = 100
        L_color = [color for i in range(len(marking.hole_name))]

        marking.set_cylinder_object(ang_obj=ang_obj, id_start=id_start, L_color=L_color)



        t += 0.01
        time.sleep(0.01)
        pass
