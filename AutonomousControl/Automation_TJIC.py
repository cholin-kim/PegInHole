import os, sys
sys.path.append(os.path.abspath("/AutonomousControl"))
import rospy
import copy
from scipy.spatial.transform import Rotation as R

from Kinematics.panda.pandaVar import gripper_len
from Pickup.detect_marker import Detect_Marker
from TJIC_Commander import TJIC_Commander

from Kinematics.panda.pandaKinematics import pandaKinematics
from Camera.eye_in_hand_param_d455 import *
from Gripper.DHGripperROS import DHGripperROS

gripper_tjic = TJIC_Commander(ns="/panda1")
camera_tjic = TJIC_Commander(ns="/panda2")

panda = pandaKinematics()
gripper = DHGripperROS()
dm = Detect_Marker(marker_size=0.012)

if not rospy.get_node_uri():
    rospy.init_node("pickup")

import time; time.sleep(1)
# 0. Open Gripper
rospy.loginfo("Releasing the Gripper.")
gripper.set_gripper(position=1000, speed=gripper.speed, force=20, initialize=False)
# exit()

########################################## Pick Up ##########################################
# 1. Move to waypoint that can see all possible poses   #<- motion scale joint motion scale
rospy.loginfo("Move to the waypoint for sensor detection.")
## should be manually set depending on the jig location
targ_q1 = np.array([0.16247389924717884, -1.0558583339004046, -0.049236402116937666, -2.7068842322709785, -0.040758531033700175, 1.6405905917277808, 0.6875652297632905])
gripper_tjic.set_joint(targ_q1, duration=2)

# targ_q1_camera = np.array()
# camera_tjic.set_joint(targ_q1_camera, duration=2)



Tb_ed1 = panda.fk(targ_q1)[0][-1]
# print(R.from_matrix(Tb_ed1[:3, :3]).as_euler('ZYX'))
# Tb_ed1[:3, :3] = R.from_euler('ZX', [-np.pi/6, np.pi]).as_matrix()
# gripper_tjic.set_Tb_ed(Tb_ed=Tb_ed1, duration=2)
# print(gripper_tjic.joint_state)
# print("Tb_ed1:", Tb_ed1)



# 2. Detect Aruco
rospy.loginfo("Detecting Sensor Pose.")
time.sleep(0.5)
detected_ids, aruco_poses = dm.detect_marker(visualize=False)
print("detected_ids:", detected_ids)
print("aruco_poses:", aruco_poses)

Tb_ee_gripper = panda.fk(gripper_tjic.joint_state)[0][-1]


def get_sensor_pos(aruco_poses):
    Tb_cam = Tb_ee_gripper @ Tee_cam

    Tb_marker0 = np.identity(4)
    Tb_marker0[:3, -1] = aruco_poses[0][:3]
    Tb_marker0 = Tb_cam @ Tb_marker0

    Tb_marker1 = np.identity(4)
    Tb_marker1[:3, -1] = aruco_poses[1][:3]
    Tb_marker1 = Tb_cam @ Tb_marker1

    Tb_marker2 = np.identity(4)
    Tb_marker2[:3, -1] = aruco_poses[2][:3]
    Tb_marker2 = Tb_cam @ Tb_marker2

    Tb_marker3 = np.identity(4)
    Tb_marker3[:3, -1] = aruco_poses[3][:3]
    Tb_marker3 = Tb_cam @ Tb_marker3

    Tb_sensor = np.identity(4)
    Tb_sensor[:3, -1] = np.mean((Tb_marker0[:3, -1], Tb_marker1[:3, -1], Tb_marker2[:3, -1], Tb_marker3[:3, -1]), axis=0)
    ### manually tuned
    Tb_sensor[:2, -1] += 0.003
    print("Tb_sensor:", Tb_sensor)

    return Tb_sensor    # only position is valid


import random
while not len(detected_ids[0]) == 4:
    Tb_ed = copy.deepcopy(Tb_ee_gripper)
    choice = random.choice([0, 1, 2, 3])
    if choice == 0:
        Tb_ed[0, -1] += 0.01
    elif choice == 1:
        Tb_ed[1, -1] += 0.01
    elif choice == 2:
        Tb_ed[0, -1] -= 0.01
    elif choice == 3:
        Tb_ed[1, -1] -= 0.01
    Tb_ee = Tb_ed
    gripper_tjic.set_Tb_ed(Tb_ed=Tb_ed, duration=1)

    rospy.sleep(0.5)
    detected_ids, aruco_poses = dm.detect_marker(visualize=False)
    print("detected_ids:", detected_ids)
    print("aruco_poses:", aruco_poses)


Tb_sensor = get_sensor_pos(aruco_poses)
Tb_sensor[:3, :3] = Tb_ed1[:3, :3]


# 3. Move to waypoint(10cm above the marker surface)
rospy.loginfo("Get ready to grasp the sensor.")
Tb_wp = copy.deepcopy(Tb_sensor)
Tb_wp[2, -1] += 0.1
gripper_tjic.set_Tb_ed(Tb_ed=Tb_wp, duration=3)
print("Tb_wp:", Tb_wp)



# 4. Open Gripper
rospy.loginfo("Opening the gripper.")
gripper.set_gripper(position=1000, speed=gripper.speed, force=20, initialize=False)


# 5. Go down & Set Gripper Ready
rospy.loginfo("Going down straight and set gripper ready to grasp.")
Tb_ed = copy.deepcopy(Tb_sensor)
Tb_ed[2, -1] += 0.007
print("Tb_ed:", Tb_ed)
gripper_tjic.set_cartesian_path(Tb_ed=Tb_ed, duration=2)
gripper.set_gripper_ready()
rospy.sleep(0.5)


# 6. Close Gipper
rospy.loginfo("Going down further and closing the gripper.")
Tb_ed = copy.deepcopy(Tb_ed)
Tb_ed[2, -1] -= 0.007
gripper_tjic.set_cartesian_path(Tb_ed=Tb_ed,duration=1)
gripper.set_gripper_grasp()
rospy.sleep(0.5)

# 7. Go up
rospy.loginfo("Sensor grasped, going up.")
Tb_ed = copy.deepcopy(Tb_ed)
Tb_ed[2, -1] += 0.1
# gripper_joint = gripper_tjic.joint_state
# Tb_ee_gripper = panda.fk(gripper_joint)[0][-1]
# Tee_ee = np.identity(4)
# Tee_ee[2, -1] = 0.1
# Tb_ed = Tb_ee_gripper @ Tee_ee
gripper_tjic.set_cartesian_path(Tb_ed=Tb_ed, duration=2)





########################################## Pick Up ##########################################