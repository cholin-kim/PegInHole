#!/usr/bin/env python
import rospy
import cv2
from Kinematics.panda.pandaKinematics import pandaKinematics
from Utils.cal_hole import *
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose, PoseStamped, Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class marker(object):
    def __init__(self):
        self.pub = rospy.Publisher('/hole_transforms', TFMessage, queue_size=10)
        self.pub2 = rospy.Publisher('/nearest_hole', Vector3, queue_size=1)

        self.bridge = CvBridge()
        self.poseStamped = PoseStamped()
        self.tf_msg = TFMessage()
        image_raw = rospy.wait_for_message("/camera/color/image_raw", Image)
        self.image = self.bridge.imgmsg_to_cv2(image_raw, 'bgr8')

        self.hole_list = ["hole_top", 'hole_1_1', 'hole_1_2','hole_1_3','hole_1_4','hole_2_1','hole_2_2','hole_2_3','hole_2_4','hole_2_5','hole_2_6','hole_2_7', 'hole_2_8','hole_3_1','hole_3_2','hole_3_3','hole_3_4','hole_3_5','hole_3_6','hole_3_7','hole_3_8','hole_4_1','hole_4_2','hole_4_3','hole_4_4','hole_4_5','hole_4_6','hole_4_7','hole_4_8']
        joint_msg = rospy.wait_for_message("/franka_state_controller/joint_states", JointState)
        self.joint_state = joint_msg.position

        self.calculated_pose = []
        self.t_list = []
        self.positions = np.array([])
        self.normals = np.array([])
        self.signal = 1

        rospy.Subscriber("/franka_state_controller/joint_states", JointState, self.joint_states_callback, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image, self.img_callback)
        rospy.Subscriber("/signal", Pose, self.signal_callback)

    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def joint_states_callback(self, message):
        self.joint_state = list(message.position)
        return

    def signal_callback(self, message):
        self.signal = message
        if self.signal == 1:
            self.cal_all_hole()

    def cal_all_hole(self):
        for i in self.hole_list:
            trans_o2h = cal_o2h(i)
            mtx_c2o = self.cal_c2ob(self.image) # cam to object base 계산
            if len(mtx_c2o) == 0:
                print('maker is not detected')
                return
            self.calculated_pose = (self.trans_base2cam_aruco(mtx_c2o.dot(trans_o2h)))
            self.t_list.append([i, self.calculated_pose])

        self.make_msg()
        self.positions = np.array([matrix[:3, 3] for _, matrix in self.t_list])
        self.normals = np.array([matrix[:3, 2] for _, matrix in self.t_list])
        print(self.t_list)

    def cal_c2ob(self, image): # cam to ob 계산하는 함수
        target1 = 1
        target2 = 2
        total_id_len = 100
        dx = 0.025
        dy = 0.025

        aruco_dict_want = 'DICT_4X4_50'
        ARUCO_DICT = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
            'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL
        }

        in_mtx = np.array([[430.583740234375, 0.0, 419.8208312988281],
              [0.0, 430.1481628417969, 239.1549835205078],
              [0.0, 0.0, 1.0]])
        in_mtx = in_mtx.astype('float32')
        #di_coeff = np.array([-0.05458524078130722, 0.057370759546756744, 0.00011702199117280543, 0.0012725829146802425, -0.018503589555621147])
        di_coeff = np.array([0.0, 0.0, 0.0, 0.0])
        di_coeff = di_coeff.astype('float32')

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dict_want])
        aruco_param = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_param)
        corners, ids, reject = detector.detectMarkers(image)

        len_ids = 0
        if ids is not None:
            ids = ids.reshape((-1))
            len_ids = len(ids)
            print('len(ids) :', len_ids)
            print('ids :', ids)
        else:
            print('len(ids) :', len_ids)
        print()

        rvec_L = [[] for i in range(total_id_len)]
        tvec_L = [[] for i in range(total_id_len)]

        for i in range(len_ids):
            objPoint = np.array([[-dx / 2, dy / 2, 0.0], [dx / 2, dy / 2, 0.0], [dx / 2, -dy / 2, 0.0], [-dx / 2, -dy / 2, 0.0]])
            objPoint = objPoint.astype('float32')
            imgPoint = corners[i]
            imgPoint = imgPoint.astype('float32')
            _, rvec, tvec = cv2.solvePnP(objPoint, imgPoint, in_mtx, di_coeff)
            rvec_L[ids[i]] = rvec
            tvec_L[ids[i]] = tvec

        t_mtx = []
        inv_t_mtx = []

        ## detected marker를 transformation matrix변환

        if len(rvec_L[1]) != 0 and len(rvec_L[2]) != 0:
            for k in range(target1, target2 + 1):
                rvec, _ = cv2.Rodrigues(rvec_L[k])
                tvec = tvec_L[k]
                trans_matrix = np.zeros((4, 4))
                for i in range(3):
                    for j in range(3):
                        trans_matrix[i][j] = rvec[i][j]
                for i in range(3):
                    tvec = tvec.reshape((-1,))
                    trans_matrix[i][3] = tvec[i]
                trans_matrix[3][3] = 1.0
                t_mtx.append(trans_matrix)
                inv_t_mtx.append( np.linalg.inv(trans_matrix) )

        if len(t_mtx) == 0 or len(inv_t_mtx) == 0:
            return []

        t_1T2 = np.matmul(inv_t_mtx[1], t_mtx[0])
        for i in range(3):
            t_1T2[i][3] = t_1T2[i][3] / 2.0

        t_result = np.matmul(t_1T2, inv_t_mtx[0])   # top2camera
        t_result = np.linalg.inv(t_result)

        mtx_o2h = cal_o2h('hole_top')
        t_result = t_result @ np.linalg.inv(mtx_o2h)

        return t_result #

    def trans_base2cam_aruco(self, mat):
        panda = pandaKinematics()
        b2e, aa = panda.fk(self.joint_state)

        print(b2e[-1])

        T_ee_cam = np.array([[0.000279179 ,    0.99999, -0.00454143,  -0.0927863],
                   [-0.999999, 0.000274307, -0.00107333,   0.0116264],
                   [-0.00107208, 0.00454173, 0.999989, 0.103118-0.18],
                   [0,           0,           0           ,1]])

        b2e = np.matmul(b2e[-1], T_ee_cam)
        Tba = np.matmul(b2e, mat)
        return Tba

    def make_msg(self):
        for hole_name, matrix in self.t_list:
            translation = matrix[:3, 3]
            rotation = matrix[:3, :3]

            quaternion = R.from_matrix(rotation).as_quat()  # [qx, qy, qz, qw]
            qx, qy, qz, qw = quaternion

            transform = TransformStamped()
            transform.header.frame_id = "base"
            transform.child_frame_id = hole_name

            transform.transform.translation.x = translation[0]
            transform.transform.translation.y = translation[1]
            transform.transform.translation.z = translation[2]
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            self.tf_msg.transforms.append(transform)

    def publish(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            panda = pandaKinematics()
            b2e, aa = panda.fk(self.joint_state)

            ee_pos = b2e[-1][:3,3]

            distances = np.linalg.norm(self.positions - ee_pos, axis=1)

            min_idx = np.argmin(distances)
            closest_hole_name = self.hole_list[min_idx]
            # print(closest_hole_name)
            nVec = self.normals[min_idx]

            normal_msg = Vector3()
            normal_msg.x = nVec[0]
            normal_msg.y = nVec[1]
            normal_msg.z = nVec[2]

            self.pub.publish(self.tf_msg)
            self.pub2.publish(normal_msg)
            rate.sleep()

    # def publish_nearest_nVec(self):
    #     rate = rospy.Rate(100)  # 10 Hz
    #
    #     while not rospy.is_shutdown():
    #         panda = pandaKinematics()
    #         b2e, aa = panda.fk(self.joint_state)
    #
    #         ee_pos = b2e[-1][:3,3]
    #
    #         distances = np.linalg.norm(self.positions - ee_pos, axis=1)
    #
    #         min_idx = np.argmin(distances)
    #         closest_hole_name = self.hole_list[min_idx]
    #         # print(closest_hole_name)
    #         nVec = self.normals[min_idx]
    #
    #         normal_msg = Vector3()
    #         normal_msg.x = nVec[0]
    #         normal_msg.y = nVec[1]
    #         normal_msg.z = nVec[2]
    #
    #         self.pub2.publish(normal_msg)
    #         rate.sleep()



if __name__ == '__main__':
    rospy.init_node("aruco_detect", anonymous=True)
    node = marker()
    a = ""

    ## 해야하는 거
    ## 1. signal 처리하는 거 코드 토픽 이름 맞춰서 작성하기

    node.cal_all_hole()
    node.publish()
    # node.publish_nearest_nVec()