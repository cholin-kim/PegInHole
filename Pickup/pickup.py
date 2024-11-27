import rospy
import numpy as np
import copy


from scipy.spatial.transform import Rotation as R

from detect_marker import Detect_Marker
from PJTC import PJTC
dm = Detect_Marker()
pjtc = PJTC()

import cv2
from pandaKinematics import pandaKinematics
import pandaVar as pandaVar
from camera_param import *

panda = pandaKinematics()

class Pickup:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node("pickup")

        # rectangle information defined by aruco markers as 4 corners
        self.w = 0.07
        self.h = 0.07

        detected_ids, aruco_poses = dm.detect_marker()
        sensor_pose = self.get_sensor_pose(detected_ids=detected_ids, aruco_poses=aruco_poses)
        self.ready_to_pick(sensor_pose)



    def get_sensor_pose(self, detected_ids, aruco_poses):
        '''
        :param detected_ids:
        :param aruco_poses:
        :return: Tcam_sensor
        '''
        if len(self, detected_ids) == 1:
            if detected_ids[0] == 0:
                sensor_pose = copy.deepcopy(aruco_poses[0])
                sensor_pose[0] += self.w/2
                sensor_pose[1] -= self.h/2

        # # position
        # # sensor_pos[2] = np.mean(aruco_poses[found_id, 2])
        # center_pos_cand = []
        # for id in found_id:
        #     center_pos_cand.append(self.center_cand(aruco_poses[id][:3]))
        # center_pos = np.mean(np.array(center_pos_cand), axis=0)
        #
        # # orientation

        Tcam_sensor = np.eye(4)
        Tcam_sensor[:3, -1] = sensor_pose[:3]
        Tcam_sensor[:3, :3] = R.from_rotvec(sensor_pose[3:]).as_matrix()

        return Tcam_sensor



    def ready_to_pick(self, Tcam_sensor):
        Tb_wp = self.get_waypoint(self, Tcam_sensor)
        pjtc.set_pose_direct(Tb_wp)


    def get_waypoint(self, Tcam_sensor):
        Tb_ee = panda.fk(pjtc.joint_state)[0][-1]
        Tb_sensor = Tb_ee @ Tee_cam @ Tcam_sensor
        Tb_wp = np.copy(Tb_sensor)
        Tb_wp[2, -1] += 0.1
        return Tb_wp













# def center_cand(self, pos, id):
#     if (id % 4) == 0:
#         return np.array([pos[0] + self.w / 2, pos[1] - self.w / 2, pos[2]])
#     elif (id % 4) == 1:
#         return np.array([pos[0] - self.w / 2, pos[1] - self.w / 2, pos[2]])
#     elif (id % 4) == 2:
#         return np.array([pos[0] + self.w / 2, pos[1] + self.w / 2, pos[2]])
#     elif (id % 4) == 3:
#         return np.array([pos[0] - self.w / 2, pos[1] + self.w / 2, pos[2]])
