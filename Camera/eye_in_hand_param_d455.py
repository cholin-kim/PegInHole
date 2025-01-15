import numpy as np




# header:
#   seq: 13800
#   stamp:
#     secs: 1736737816
#     nsecs: 342044592
#   frame_id: "camera_color_optical_frame"
# height: 480
# width: 640
# distortion_model: "plumb_bob"
# D: [-0.0536506250500679, 0.06849482655525208, -0.0003799688129220158, 0.0006371866911649704, -0.021824028342962265]
# K: [383.37890625, 0.0, 329.32598876953125, 0.0, 382.80999755859375, 243.7167510986328, 0.0, 0.0, 1.0]
# R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P: [383.37890625, 0.0, 329.32598876953125, 0.0, 0.0, 382.80999755859375, 243.7167510986328, 0.0, 0.0, 0.0, 1.0, 0.0]
# binning_x: 0
# binning_y: 0
# roi:
#   x_offset: 0
#   y_offset: 0
#   height: 0
#   width: 0
#   do_rectify: False
# ---


# sensor_msgs/CameraInfo.msg
'''
The internal parameters can be used to warp a raw(=distorted) image to:
    1. An undistorted img(requires D and K)
    2. A rectified img(requires D, K, R)
'''
img_width = 640
img_height = 480


# Distortion Parameters
D = np.array([-0.0536506250500679, 0.06849482655525208, -0.0003799688129220158, 0.0006371866911649704, -0.021824028342962265])

# Intrinsic matrix
# [[f_x, 0, c_x],
# [0, f_y, c_y],
# [0, 0, 1]]
K = np.array([[383.37890625, 0.0,  329.32598876953125],
              [0.0, 382.809997558593759, 243.7167510986328],
              [0.0, 0.0, 1.0]])

# Projection matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
P = np.array([[383.37890625, 0.0, 329.32598876953125, 0.0],
              [0.0, 382.80999755859375, 243.7167510986328, 0.0],
              [0.0, 0.0, 1.0, 0.0]])

from Kinematics.panda.pandaVar import *

# Tee_cam =np.array([[0.000279179 ,    0.99999, -0.00454143,  -0.0927863],
#                    [-0.999999, 0.000274307, -0.00107333,   0.0116264],
#                    [-0.00107208, 0.00454173, 0.999989, 0.103118-gripper_len],
#                    [0,           0,           0           ,1]])
pose_flange_cam = np.array([0.0120477, -0.0955359, 0.0829928,   -0.00207123, -0.00146682, 0.00180441, 0.999995])
from scipy.spatial.transform import Rotation as R

Tflange_cam = np.eye(4)
Tflange_cam[:3, :3] = R.from_quat(pose_flange_cam[3:]).as_matrix()
Tflange_cam[:3, -1] = pose_flange_cam[:3]

Tflange_ee = np.identity(4)
Tflange_ee[2, -1] = gripper_len

Tee_cam = np.linalg.inv(Tflange_ee) @ Tflange_cam





# R.from_matrix(Tee_cam[:3,:3]).as_euler('XYZ', degrees=True)
# array([ -0.16813161,   0.61100795, 135.04031963]) -> calibration verified
