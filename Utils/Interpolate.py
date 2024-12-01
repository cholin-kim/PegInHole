import numpy as np

from Kinematics.panda.pandaKinematics import pandaKinematics
panda = pandaKinematics()
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline
from scipy.spatial.transform import Slerp


def interpolate_q(q_lst, duration, visualize=False):
    '''
    performs cubic spline interpolation.
    limitation: do not allow us to specify the initial or final acceleration
    assumption: each point in q_lst has same time distance
    :param q_lst: waypoints of trajectory
    :return:
    '''

    t = np.linspace(0, duration, len(q_lst))

    # Too many sampling does not allow robot reaching its interpolated waypoints before the next goal, hence worse performance
    ts = np.linspace(0, duration, min(int(duration * 10), len(q_lst)))


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
            plt.scatter(np.ones(len(q_lst[0])) * t[i], q_lst[i], s=20, c='red')
        plt.show()

    q_traj = np.array(q_traj).reshape(7, -1)

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

    ori = R.from_matrix(start_T[:3, :3])
    ori_des = R.from_matrix(end_T[:3, :3])

    # slerp = Slerp(t, R.concatenate([ori, ori_des]))
    RS = RotationSpline(t, R.concatenate([ori, ori_des]))   # rotation vector + cubic spline
    # Limitation: The angular acceleration is continuous, but not smooth.

    ts = np.linspace(0, duration, int(duration * 30))
    # import pdb;pdb.set_trace()

    pos_traj = np.linspace(pos, pos_des, len(ts))
    ori_traj = RS(ts).as_matrix()
    # ori_traj = slerp(ts).as_matrix()
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
    # ts, q_traj = interpolate_q_modf([q0, q1, q2, q3], duration, visualize=True)


    Tb_ed = np.array([
        [ 1.     ,  0.     ,  0.     ,  0.55233],
        [ 0.     , -1.     , -0.     ,  0.06299],
        [ 0.     ,  0.     , -1.     ,  0.37146],
        [ 0.     ,  0.     ,  0.     ,  1.     ]])

    Tb_ee = np.array([
        [0.94829, 0.16114, 0.27347, 0.38523],
        [0.18755, -0.97953, -0.07316, 0.12156],
        [0.25608, 0.12067, -0.95909, 0.6306],
        [0., 0., 0., 1.]])

    Ts = interpolate_T(Tb_ee, Tb_ed, duration=7)