import rospy
import numpy as np
import copy

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from Kinematics.panda.pandaKinematics import pandaKinematics

panda = pandaKinematics()



class PJTC: # position joint trajectory controller
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node("position_joint_trajectory_controller")
        self.action = rospy.resolve_name('~follow_joint_trajectory')
        self.client = SimpleActionClient(self.action, FollowJointTrajectoryAction)
        rospy.loginfo("PJTC: Waiting for '" + self.action + "' action to come up")
        self.client.wait_for_server()


        joint_state_topic = "/franka_state_controller/joint_states"
        rospy.loginfo("PJTC: Waiting for message on topic '" + joint_state_topic + "'")
        self.joint_state = rospy.wait_for_message(joint_state_topic, JointState)
        self.joint_name = self.joint_state.name
        self.cur_q = self.joint_

        self.cur_pose = dict(zip(self.joint_state.name, self.joint_state.position))

        self.fr3_max_dq = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
        # self.targ_pose = copy.deepcopy(self.cur_pose)

    def set_joint_direct(self, targ_q=None):
        '''
        :param target_q(list)
        '''

        target_pose = copy.deepcopy(self.cur_pose)
        if targ_q is None:
            for key in target_pose:
                target_pose[key] += float(0.05 * np.random.randint(-1, 2))

        else:
            i =0
            for key in target_pose:
                target_pose[key] = targ_q[i]
                i += 1

        print("current_pose:", self.cur_pose)
        print("target_pose:", target_pose)




        max_movement = max(abs(target_pose[joint] - self.cur_pose[joint]) for joint in target_pose)

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(
            # Use either the time to move the furthest joint with 'max_dq' or 500ms,
            # whatever is greater
            max(max_movement / rospy.get_param('~max_dq', 0.5), 0.5)
        )

        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*target_pose.items())]
        point.velocities = [0] * len(target_pose)

        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)

        rospy.loginfo('Sending trajectory Goal to target config')
        self.client.send_goal_and_wait(goal)

        result = self.client.get_result()

        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logerr('PJTC: Movement was not successful: ' + {
                FollowJointTrajectoryResult.INVALID_GOAL:
                """
                The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
                Is the 'joint_pose' reachable?
                """,

                FollowJointTrajectoryResult.INVALID_JOINTS:
                """
                The joint pose you specified is for different joints than the joint trajectory controller
                is claiming. Does you 'joint_pose' include all 7 joints of the robot?
                """,

                FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
                """
                During the motion the robot deviated from the planned path too much. Is something blocking
                the robot?
                """,

                FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                """
                After the motion the robot deviated from the desired goal pose too much. Probably the robot
                didn't reach the joint_pose properly
                """,
            }[result.error_code])

        else:
            rospy.loginfo('PJTC: Successfully moved into target pose')

    def set_pose_direct(self, targ_T = None):





if __name__ == "__main__":
    pjtc = PJTC()
    while True:
        pjtc.move_to_target_direct()
