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
            self.cur_q = None
            self.fr3_max_dq = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])

            self.joint_pub = rospy.Publisher(self.global_ns + self.ns + "/joint_states", JointState, queue_size=1)

            self.joint_names = rospy.wait_for_message(self.ns + "/joint_states", JointState).name
            rospy.Subscriber(self.ns + "/joint_states", JointState, self.joint_cb)
            rospy.wait_for_message(self.ns + "/joint_states", JointState, timeout=5)



    def joint_cb(self, msg:JointState):
        self.joint_state = msg.position

    def set_joint_direct(self, targ_q):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = targ_q
        print(msg)
        # self.joint_pub.publish(msg)

    def set_joint(self, targ_q_lst, duration):
        ts, q_traj = interpolate_q(targ_q_lst, duration)

        for q in q_traj:
            self.set_joint_direct(targ_q=q)
            print(ts[1] - ts[0])
            # rospy.sleep(ts[1] - ts[0])
            rospy.sleep(rospy.Duration.from_sec(1))


########################################################################################################################
    # def set_cartesian(self, Tb_ed, duration, Tb_ee=None, force_duration=True):
    #     if Tb_ee is None:
    #         Tb_ee = panda.fk(self.joint_state.position)[0][-1]
    #     q_des = panda.ik(Tb_ed, q0=self.joint_state.position)
    #     q_distance = np.abs(q_des - self.joint_state.position)
    #
    #     duration_exe = self.get_duration(duration_des=duration, q_distance=q_distance, force_duration=force_duration)
    #
    #     ts, q_traj = interpolate_T(start_T=Tb_ee, end_T=Tb_ed, duration=duration_exe)
    #
    #
    # def get_duration(self, duration_des, q_distance, force_duration=True):
    #     duration_base = max(max(q_distance / self.fr3_max_dq), 1) * 3   # 3 sec as default
    #
    #     if force_duration:
    #         duration_exe = duration_des
    #     else:
    #         if duration_des > duration_base:
    #             duration_exe = duration_base
    #         else:
    #             duration_exe = duration_des
    #     print("duration_chosen:", duration_exe)
    #     return duration_exe


if __name__ == "__main__":
    gripper_tjic = TJIC_Commander(ns="/panda1")

    t = 0.0
    while not rospy.is_shutdown():
        targ_q = copy.deepcopy(gripper_tjic.joint_state)

        ## 1. set_joint_direct test
        # targ_q += np.ones(7) * t
        # gripper_tjic.set_joint_direct(targ_q=targ_q)
        # t += 0.01

        ## 2. set_joint test
        targ_q += np.ones(7) * 0.1
        gripper_tjic.set_joint(targ_q_lst=[gripper_tjic.cur_q, targ_q], duration=5)
        exit()



        rospy.sleep(1)