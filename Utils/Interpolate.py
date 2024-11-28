import numpy as np

from Kinematics.panda.pandaKinematics import pandaKinematics
panda = pandaKinematics()
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline



def interpolate_q(start_q, end_q, duration):
    t = np.array([0, duration])
    cs = CubicSpline(t, np.array([start_q, end_q]), bc_type='natural')

    ts = np.linspace(0, duration, int(duration * 50))
    q_traj = cs(ts)
    return q_traj


def interpolate_T(start_T, end_T, duration):
    t = np.array([0, duration])
    pos = start_T[:3, -1]
    pos_des = end_T[:3, -1]

    cs = CubicSpline(t, np.array([pos, pos_des]), bc_type='natural')

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