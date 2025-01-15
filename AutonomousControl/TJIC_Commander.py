# Usage: surglab controllers Teleop Joint Impedance Controller Commander
import rospy
from sensor_msgs.msg import JointState
import copy
import numpy as np
from Utils.Interpolate import *



class TJIC_Commander:
    def __init__(self, ns):
        if not rospy.get_node_uri():
            rospy.init_node("TJTC_Commander", anonymous=True)

            self.global_ns = "/panda_sim"
            self.ns = ns
            self.joint_state = None
            self.fr3_max_dq = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])

            self.joint_pub = rospy.Publisher(self.global_ns + self.ns + "/joint_states", JointState, queue_size=1)

            self.joint_names = rospy.wait_for_message(self.ns + "/joint_states", JointState).name
            rospy.Subscriber(self.ns + "/joint_states", JointState, self.joint_cb)
            rospy.wait_for_message(self.ns + "/joint_states", JointState, timeout=5)



    def joint_cb(self, msg:JointState):
        self.joint_state = np.array(msg.position)

    def set_joint_direct(self, targ_q):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = targ_q
        self.joint_pub.publish(msg)


    def set_joint(self, targ_q, duration):
        ts, q_traj = interpolate_q([self.joint_state, targ_q], duration)

        for q in q_traj:
            self.set_joint_direct(targ_q=q)
            rospy.sleep(ts[1] - ts[0])

    def set_Tb_ed(self, Tb_ed, duration):
        targ_q = panda.ik(Tb_ed, q0=self.joint_state)
        self.set_joint(targ_q=targ_q, duration=duration)



    def set_cartesian_path(self, Tb_ed, duration, Tb_ee=None, force_duration=True):
        if Tb_ee is None:
            Tb_ee = panda.fk(self.joint_state)[0][-1]
        q_des = panda.ik(Tb_ed, q0=self.joint_state)
        q_distance = np.abs(q_des - self.joint_state)

        duration_exe = self.get_duration(duration_des=duration, q_distance=q_distance, force_duration=force_duration)

        ts, Ts = interpolate_T(start_T=Tb_ee, end_T=Tb_ed, duration=duration_exe, visualize=False)
        q_traj = []

        for T_ in Ts:
            q_traj.append(panda.ik(T_, self.joint_state))

        for q in q_traj:
            self.set_joint_direct(targ_q=q)
            rospy.sleep(ts[1] - ts[0])



    def get_duration(self, duration_des, q_distance, force_duration=True):
        duration_base = max(max(q_distance / self.fr3_max_dq), 1) * 3   # 3 sec as default

        if force_duration:
            duration_exe = duration_des
        else:
            if duration_des > duration_base:
                duration_exe = duration_base
            else:
                duration_exe = duration_des
        print("duration_chosen:", duration_exe)
        return duration_exe


if __name__ == "__main__":

    gripper_tjic = TJIC_Commander(ns="/panda1")

    import time
    time.sleep(3)

    t = 0.0

    targ_q = copy.deepcopy(gripper_tjic.joint_state)
    cur_T = panda.fk(targ_q)[0][-1]
    targ_T = np.copy(cur_T)



    ## 1. set_joint_direct test
    # targ_q[-1] += 0.1
    # gripper_tjic.set_joint_direct(targ_q=targ_q)
    # t += 0.01
    # quit()

    # 2. set_joint test
    targ_q[-1] -= 0.1
    gripper_tjic.set_joint(targ_q=targ_q, duration=3)


    ## 3. set_cartesian_path test
    # Tee2ee = np.eye(4)
    # Tee2ee[:3, :3] = R.from_euler('X', [10], degrees=True).as_matrix()
    # targ_T = targ_T @ Tee2ee
    # gripper_tjic.set_cartesian_path(Tb_ed=targ_T, duration=5)