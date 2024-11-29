import numpy as np

from Kinematics.panda.pandaKinematics import pandaKinematics
panda = pandaKinematics()
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline



def interpolate_q(start_q, end_q, duration):
    t = np.array([0, duration])
    cs = CubicSpline(t, np.array([start_q, end_q]), bc_type='clamped')

    ts = np.linspace(0, duration, int(duration * 100))
    q_traj = cs(ts)
    return ts, q_traj

# def interpolate_q_(q_lst, duration):
#     t = np.linspace(0, duration, len(q_lst))
#     cs = CubicSpline(t, np.array(q_lst), bc_type='clamped')
#
#     ts = np.linspace(0, duration, int(duration * 100))
#     q_traj = cs(ts)
#     return ts, q_traj
#
# from roboticstoolbox import trapezoidal
# def interpolate_q_lspb(start_q, end_q, duration, max_vel=None):
#     ts = np.linspace(0, duration, int(duration * 100))
#     q_traj = np.zeros((len(ts), len(start_q)))
#
#     for i in range(len(start_q)):
#         lspb = trapezoidal(start_q[i], end_q[i], t=len(ts))
#         q_traj[:, i] = lspb.q
#
#     return ts, q_traj




def interpolate_T(start_T, end_T, duration):
    t = np.array([0, duration])
    pos = start_T[:3, -1]
    pos_des = end_T[:3, -1]

    cs = CubicSpline(t, np.array([pos, pos_des]), bc_type='clamped')

    ori = R.from_matrix(start_T[:3, :3])
    ori_des = R.from_matrix(end_T[:3, :3])

    RS = RotationSpline(t, R.concatenate([ori, ori_des]))

    ts = np.linspace(0, duration, int(duration * 100))
    pos_traj = cs(ts)
    ori_traj = RS(ts).as_matrix()
    Ts = np.zeros((len(pos_traj), 4, 4))
    Ts[:, -1, -1] = 1
    Ts[:, :3, -1] = pos_traj
    Ts[:, :3, :3] = ori_traj
    return Ts