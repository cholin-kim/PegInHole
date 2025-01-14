import rospy
from sensor_msgs.msg import JointState
import numpy as np
import tf

# Used temporarily to set base pose of panda_lift_base. This code will be substituted to base pose estimation code via ICP or marker


# if not rospy.get_node_uri():
rospy.init_node("pub_base_pose")


tf_broader = tf.TransformBroadcaster()


'''
Input
lift_stroke
curved...
'''
lift_stroke = 0.5

t = 0.0
ang = 0.5 + 1.5 * np.cos(0.15 * np.pi * t)
r = -0.625


def set_lift_stroke_tf(pos_xyz, rot_euler, parent='cart_base', child='lift_base'):
    tf_broader.sendTransform(
        translation=pos_xyz,
        rotation = tf.transformations.quaternion_from_euler(rot_euler[0], rot_euler[1], rot_euler[2]),
        time=rospy.Time.now(),
        parent=parent,
        child=child
    )

def set_lift_tf(pos_xyz, rot_euler, parent='base', child='cart_base'):
    tf_broader.sendTransform(
        translation=pos_xyz,
        rotation=tf.transformations.quaternion_from_euler(rot_euler[0], rot_euler[1], rot_euler[2]),
        time=rospy.Time.now(),
        parent=parent,
        child=child
    )

while not rospy.is_shutdown():
    # set_lift_stroke_tf(pos_xyz=[0, 0, lift_stroke], rot_euler=[0, 0, 0])
    set_lift_tf(pos_xyz=[0.3, 0.3, 0.3], rot_euler=[0, 0, t])
    t += 0.01

    rospy.Rate(1).sleep()
    print(t)
