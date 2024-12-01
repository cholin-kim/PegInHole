import numpy as np

from Kinematics.panda.pandaKinematics import pandaKinematics
panda = pandaKinematics()
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline



def interpolate_q(q_lst, duration, visualize=False):
    '''
    performs cubic spline interpolation.
    limitation: do not allow us to specify the initial or final acceleration
    assumption: each point in q_lst has same time distance
    :param q_lst: waypoints of trajectory
    :return:
    '''
    t = np.linspace(0, duration, len(q_lst))
    ts = np.linspace(0, duration, int(duration * 100))

    q_lst = np.array(q_lst).reshape(len(q_lst), -1)


    cs = []
    q_traj = []

    for i in range(len(q_lst[0])):
        cs.append(CubicSpline(t, q_lst[:, i], bc_type='clamped'))

        q_traj_individual = cs[i](ts)
        q_traj.append(q_traj_individual)

    if visualize:
        import matplotlib.pyplot as plt
        for i in range(len(q_lst[0])):
            plt.scatter(ts, q_traj[i], s=2)
        for i in range(len(t)):
            plt.scatter(np.ones(len(q_lst[0])) * t[i], q_lst[i], s=20)
        plt.show()

    return ts, q_traj

# def interpolate_q_modf(q_lst, duration, visualize=False):
#     '''
#     modified version of interpolate_q, guarantees start vel and end vel as 0.
#     '''
#
#
#     t = np.linspace(0, duration, len(q_lst))
#     t = np.append(0, t) <-- error: t should increase monotonically
#     t = np.append(t, t[-1])
#
#     ts = np.linspace(0, duration, int(duration * 100))
#     ts = np.append(0, ts)
#     ts = np.append(ts, ts[-1])
#
#     q_lst = np.array(q_lst).reshape(len(q_lst), -1)
#     q_lst = np.concatenate((q_lst[0].reshape(1, -1), q_lst), axis=0)
#     q_lst = np.concatenate((q_lst, q_lst[-1].reshape(1, -1)), axis=0)
#
#     cs = []
#     q_traj = []
#
#     for i in range(len(q_lst[0])):
#         import pdb;pdb.set_trace()
#         cs.append(CubicSpline(t, q_lst[:, i], bc_type='clamped'))
#
#         q_traj_individual = cs[i](ts)
#         q_traj.append(q_traj_individual)
#
#     if visualize:
#         import matplotlib.pyplot as plt
#         for i in range(len(q_lst[0])):
#             plt.scatter(ts, q_traj[i], s=2)
#         for i in range(len(t)):
#             plt.scatter(np.ones(len(q_lst[0])) * t[i], q_lst[i], s=20)
#         plt.show()
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


if __name__ == "__main__":
    q0 = np.array([0, 0, 0, 0, 0, 0])
    q1 = np.array([2, 3, 4, 5, 6, 10])
    q2 = np.array([2, -1, 1, 5, 0, 15])
    q3 = np.array([10, 1, 2, 10, -2, 0])
    duration = 4

    t = np.array([0, 1, 2, 3])
    q_lst = [q0, q1, q2, q3]
    q_lst = np.array(q_lst).reshape(len(q_lst), -1)

    # ts, q_traj = interpolate_q([q0, q1, q2, q3], duration, visualize=True)
    ts, q_traj = interpolate_q_modf([q0, q1, q2, q3], duration, visualize=True)
