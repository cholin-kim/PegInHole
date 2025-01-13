import rospy
import numpy as np
from sensor_msgs.msg import JointState
import time
import tf
import os
# from panda.utils.Trajectory import Trajectory
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
np.set_printoptions(precision=5, suppress=True)
path = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]



class pandaRvizMotionTeleassembly:
    def __init__(self, ns='panda_lift', q0=None):
        if q0 is None:
            if ns == "panda_lift":
                q0 = [
                    0,
                    0.0330, 0.3160, 0.3840, 0.0880, 0.1070, 0.0, 0.0,
                    0, 0, 0,
                    0.6330, 0.3160, 0.3840, 0.0880, 0.1070, 0.0, 0.0
                ]
            elif ns == "environment":
                q0 = [0.0]

        self.ns = ns

        # ROS subscriber
        rospy.Subscriber('/panda_sim/' + self.ns + '/joint_states', JointState, self.joint_states_callback, queue_size=1)

        # ROS publisher
        self._set_joint_pub = rospy.Publisher('/panda_sim/' + self.ns + '/joint_states', JointState, latch=True, queue_size=1)
        self._set_pose_pub = rospy.Publisher('/panda_sim/' + self.ns + '/pose_states', PoseStamped, latch=True, queue_size=1)
        self.tf_broader = tf.TransformBroadcaster()

        # Robot Configurations
        self.max_dq = 0.5
        self.joints = q0
        self._joint_names = self.get_joint_name(suffix='')

        # Initialize Robot's pose (publish topic 3 times)
        self.set_joint_position_direct(joints=self.joints)
        time.sleep(0.1)
        self.set_joint_position_direct(joints=self.joints)
        time.sleep(0.1)
        self.set_joint_position_direct(joints=self.joints)

        # Initialize ROS node
        if not rospy.get_node_uri():
            rospy.init_node(self.ns + "_rviz_motion", anonymous=True)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

    """
    Get functions
    """
    def get_joint_name(self, suffix=''):
        joint_names = []
        if self.ns == "panda_lift":
            # joint_names.append(suffix + "joint_base")
            joint_names.append(suffix + "joint_lift_base")

            for i in range(1, 7 + 1):
                joint_names.append(suffix + "panda2_joint" + str(i))

            for i in range(1, 3 + 1):
                joint_names.append(suffix + "panda2_joint_finger" + str(i))

            for i in range(1, 7 + 1):
                joint_names.append(suffix + "fr3_joint" + str(i))

        elif self.ns == "environment":
            joint_names.append(suffix + "object")

        return joint_names


    def joint_states_callback(self, msg):
        # if self.ns == "panda_lift":
        self.joints = list(msg.position)


    """
    Set functions
    """

    def set_joint_position_direct(self, joints):    # 흠... 이거는 고려 안했는데
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self._joint_names
        # if self.ns == "panda_lift":
        msg.position = joints
        self._set_joint_pub.publish(msg)

    def set_pose_direct(self, pose):    # 흠... 이거는 고려 안하고 xarco 만들었는데...
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        if self.ns == "panda2":
            pos = pose[:3, -1]
            quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
        else:
            raise ValueError
        self._set_pose_pub.publish(msg)

    def set_joint_position(self, joints, t_step=0.01):  # 흠... trajectory.py 안쓰는 줄 알고 버렸는데...
        """
        Set position with cubic interpolation
        :param joints:  q1, q2, q3, ..., q7, pitch, yaw, jaw
        """
        q0 = self.joints
        qf = joints
        max_movement = np.max(np.abs(np.array(qf) - np.array(q0)))
        tf = max_movement / self.max_dq
        if tf < 0.5:
            tf = 0.5
        q_pos, t = Trajectory.quintic(q0=q0, qf=qf, v0=0.0, vf=0.0, a0=0.0, af=0.0, tf=tf, t_step=t_step)
        # q_pos, t = Trajectory.cubic(q0=q0, qf=qf, v0=0.0, vf=0.0, tf=tf, t_step=t_step)
        for q in q_pos:
            self.set_joint_position_direct(joints=q)
            time.sleep(t_step)

    def set_lift_tf(self, pos_xyz, rot_euler, parent='base', child='cart_base'):
        self.tf_broader.sendTransform(
            translation=pos_xyz,
            rotation=tf.transformations.quaternion_from_euler(rot_euler[0], rot_euler[1], rot_euler[2]),
            time=rospy.Time.now(),
            parent=parent,
            child=child
        )





if __name__ == "__main__":
    import time
    from panda.ros.pandaRviz import pandaRviz               # 얘는 어디에 있는거지? ... 어차피 이 파일을 메인으로 쓰지 않을거니까 상관 없나?
    from panda.ros.pandaRvizMotion import pandaRvizMotion
    from panda_teleassembly.ros.pandaRvizTeleassembly import pandaRvizTeleassembly
    sim = pandaRvizTeleassembly(is_core=False, use_GUI=False)
    panda_lift = pandaRvizMotionTeleassembly(ns='panda_lift')
    panda_lift = pandaRvizMotion(ns='panda_lift')
    t = 0.0
    while True:
        pass
        # q_des1 = [np.pi/10, 0.0, 0.0, -1.0, np.pi/3, 1.5, 0.0]
        # panda1.set_joint_position(joints=q_des1)
        # panda2.set_joint_position(joints=q_des1)
        # fr3.set_joint_position(joints=q_des1)
        #
        # q_des2 = [-np.pi/10, 0.0, 0.0, -1.0, -np.pi/3, 1.5, 0.0]
        # panda1.set_joint_position(joints=q_des2)
        # panda2.set_joint_position(joints=q_des2)
        # fr3.set_joint_position(joints=q_des2)