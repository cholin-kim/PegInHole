import rospy
import numpy as np
import copy

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

from Kinematics.panda.pandaKinematics import pandaKinematics
from Kinematics.panda import pandaVar
from Utils.Interpolate import *

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
        self.fr3_max_dq = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


    def client_send_q_traj(self, ts, q_traj):   # is this the best choice? ts and q_traj as input?
        ## Experimental
        # ts = np.append(0, ts)
        # ts = np.append(ts, ts[-1])

        # q_traj = np.concatenate((q_traj[0].reshape(1, -1), q_traj), axis=0)
        # q_traj = np.concatenate((q_traj, q_traj[-1].reshape(1, -1)), axis=0)
        ##

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = self.joint_state.name

        # time_step = ts[1] - ts[0]

        for i in range(len(q_traj)):
            point = JointTrajectoryPoint()
            point.positions = q_traj[i]
            point.time_from_start = rospy.Duration.from_sec(ts[i])  # time_from_start is relative to trajectory.header.stamp
            # point.time_from_start = rospy.Duration.from_sec(time_step)

            ## Experimental
            # if i == 0:
                # point.velocities = np.zeros(len(q_traj))
            # if i == len(q_traj) - 1:
                # point.velocities = np.zeros(len(q_traj))
            ##

            goal.trajectory.points.append(point)

        goal.goal_time_tolerance = rospy.Duration.from_sec(duration)
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
        return result


    def set_joint(self, q_des, duration=None):
        max_movement = np.abs(q_des - self.joint_state.position)

        if duration is None:
            duration = max(max_movement / self.fr3_max_dq)
            if duration < 1:
                duration = 1

        ts, q_traj = interpolate_q([self.joint_state.position, q_des], duration=duration)
        self.client_send_q_traj(ts, q_traj)



    # def set_joint_direct(self, q_des=None):
    #     '''
    #     :param target_q(list)
    #     '''
    #     target_pose = JointState()
    #     if q_des is None:
    #         target_pose.position = self.joint_state.position
    #
    #     else:
    #         target_pose.position = q_des
    #
    #
    #     # print("current_pose:", self.joint_state.position)
    #     # print("target_pose:", q_des)
    #
    #     point = JointTrajectoryPoint()
    #     goal = FollowJointTrajectoryGoal()
    #
    #     goal.trajectory.joint_names = self.joint_state.name
    #     point.positions = target_pose.position
    #     # point.velocities = [0] * len(q_des)
    #
    #     goal.trajectory.points.append(point)
    #     goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)
    #
    #
    #     q_movement = abs(q_des - self.joint_state.position)
    #
    #     duration = max(max(q_movement / self.fr3_max_dq), 0.5)
    #     point.time_from_start = rospy.Duration.from_sec(duration)
    #     # the robot should reach the specified joint positions (and other parameters) 00 seconds after the trajectory execution begins.
    #
    #
    #     rospy.loginfo('Sending trajectory Goal to target config')
    #     # self.client.send_goal_and_wait(goal)
    #     self.client.send_goal(goal)




    def set_cartesian(self, Tb_ed, Tb_ee=None, duration=None):
        if Tb_ee is None:
            Tb_ee = panda.fk(self.joint_state.position)[0][-1]
        q_des = panda.ik(Tb_ed, q0=self.joint_state.position)
        q_distance = np.abs(q_des - self.joint_state.position)

        if duration is None:
            duration = max(q_distance / self.fr3_max_dq)

        Ts = interpolate_T(start_T=Tb_ee, end_T=Tb_ed, duration=duration)
        q_cart_wp = []
        for T in Ts:
            q_cart_wp.append(panda.ik(T, q0=self.joint_state.position))

        ts, q_traj = interpolate_q(q_cart_wp, duration=duration)
        self.client_send_q_traj(ts, q_traj)






if __name__ == "__main__":
    import time
    pjtc = PJTC()
    cur_q = pjtc.joint_state.position

    ## Joint Space Command
    # targ_q = cur_q - 0.05 * np.ones(7)
    # print(cur_q)
    # print(targ_q)
    # # pjtc.set_joint_direct(q_des=targ_q)
    # pjtc.set_joint(q_des=targ_q, duration=3)
    # exit()

    ## Cartesian Space Command
    cur_T = panda.fk(cur_q)[0][-1]
    targ_T = np.eye(4)
    targ_T[:3, -1] = cur_T[:3, -1] + np.array([0.05, 0.05, 0])
    targ_T[:3, :3] = cur_T[:3, :3]
    # print(cur_T)
    # print(targ_T)
    # # pjtc.set_cartesian_direct(Tb_ed=targ_T, duration=5)
    st = time.time()
    pjtc.set_cartesian(Tb_ed=targ_T, Tb_ee=cur_T, duration=2)
    print(time.time() - st)

