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

            self.joint_pub = rospy.Publisher(self.global_ns + self.ns + "/joint_states", JointState, queue_size=1)

            self.joint_names = rospy.wait_for_message(self.ns + "/joint_states", JointState).name
            rospy.Subscriber(self.ns + "/joint_states", JointState, self.joint_cb)
            rospy.wait_for_message(self.ns + "/joint_states", JointState, timeout=5)


    def joint_cb(self, msg:JointState):
        self.cur_q = msg.position

    def set_joint_direct(self, targ_q):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = targ_q
        self.joint_pub.publish(msg)

    def set_joint(self, targ_q_lst, duration):
        ts, q_traj = interpolate_q(targ_q_lst, duration)






if __name__ == "__main__":
    gripper_tjic = TJIC_Commander(ns="/panda1")

    t = 0.0
    while not rospy.is_shutdown():
        targ_q = copy.deepcopy(gripper_tjic.cur_q)
        targ_q += np.ones(7) * t
        gripper_tjic.set_joint_direct(targ_q=targ_q)

        t+=0.01

        rospy.sleep(1)