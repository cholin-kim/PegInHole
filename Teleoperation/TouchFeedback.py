import sys
sys.path.append('.')

import numpy as np
import rospy
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import Vector3, WrenchStamped
from sensor_msgs.msg import JointState
from Kinematics.omni.omniKinematics import omniKinematics
from scipy.spatial.transform import Rotation as R

from franka_msgs.msg import FrankaState

class TouchFeedback:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('touch_feedback', anonymous=True)
        self.feedback_msg = OmniFeedback()
        self.force_feedback_msg = Vector3()
        self.omni_kin = omniKinematics()
        
        self.pub_force_feedback = rospy.Publisher('/omni2/omni/force_feedback', OmniFeedback, queue_size=1)
        self.cur_pose = self.omni_kin.fk(rospy.wait_for_message('/omni2/omni/joint_states', JointState).position)[0][-1]
        rospy.Subscriber('/omni2/omni/joint_states', JointState, callback = self.omni_cb)

        ########################### test ##########################
        rospy.Subscriber('/panda2/franka_state_controller/F_ext', WrenchStamped, callback=self.panda_force_cb)
        self.master_force = np.zeros(3)
        ###########################################################

    def omni_cb(self, msg:JointState):
        self.cur_pose = self.omni_kin.fk(msg.position)[0][-1]
        # print(self.cur_pose)

        ########################### test ##########################
    def panda_force_cb(self, msg:WrenchStamped):
        # force = [msg_force.wrench.force.x, msg_force.wrench.force.y, msg_force.wrench.force.z]
        force = [0, 0, msg.wrench.force.z]
        self.master_force = force

    def get_mtx_b2ee(self):
        msg_b2ee = rospy.wait_for_message('/panda2/franka_state_controller/franka_states', FrankaState)
        mtx_b2ee = np.transpose(np.reshape(msg_b2ee.O_T_EE, (4, 4)))
        return mtx_b2ee
        ###########################################################
    
    def publish_force_feedback(self, init_flag=False):
        feedback_force = self.master_force
        feedback_force[2] = -feedback_force[2]

        rotation = self.get_mtx_b2ee()[:3, :3]
        base_force = np.dot(rotation, feedback_force)
        scaled_base_force = R.from_euler('z', [np.pi/2]).as_matrix()[0] @ base_force
        if not init_flag:
            scaled_base_force *= 0.3
        else:
            scaled_base_force *= 0

        print(scaled_base_force)

        self.force_feedback_msg.x, self.force_feedback_msg.y, self.force_feedback_msg.z = scaled_base_force
        self.feedback_msg.force = self.force_feedback_msg
        self.pub_force_feedback.publish(self.feedback_msg)

    
    def compute_target_pose(self):
        pass


if __name__ == "__main__":
    touch_feedback = TouchFeedback()
    
    while True:
        touch_feedback.publish_force_feedback()
        rospy.sleep(0.01)