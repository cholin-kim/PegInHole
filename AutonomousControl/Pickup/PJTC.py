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
        # self.joint_name = self.joint_state.name
        # self.cur_q = self.joint_state

        # self.cur_pose = dict(zip(self.joint_state.name, self.joint_state.position))

        self.fr3_max_dq = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
        # self.targ_pose = copy.deepcopy(self.cur_pose)


    def set_joint(self, q_des):
        max_movement = np.abs(q_des - self.joint_state.position)
        duration = max(max_movement / self.fr3_max_dq)
        if duration < 1:
            duration = 1
        q_traj = interpolate_q(start_q=self.joint_state.position, end_q=q_des, duration=duration)


        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_state.name
        for q in q_traj:
            print(q)
            point = JointTrajectoryPoint()
            point.positions = q

            goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)

        result = self.client.get_result()

    def set_joint_direct(self, q_des=None):
        '''
        :param target_q(list)
        '''


        target_pose = JointState()
        # target_pose.name = self.joint_state.name
        if q_des is None:
            target_pose.position = self.joint_state.position
            # for key in target_pose:
                # target_pose[key] += float(0.05 * np.random.randint(-1, 2))
        else:
            target_pose.position = q_des
            # i =0
            # for key in target_pose:
            #     target_pose[key] = q_des[i]
            #     i += 1

        print("current_pose:", self.joint_state.position)
        print("target_pose:", q_des)

        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = self.joint_state.name
        point.positions = target_pose.position
        # goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*target_pose.items())]
        point.velocities = [0] * len(q_des)

        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)


        # max_movement = max(abs(target_pose[joint] - self.cur_pose[joint]) for joint in target_pose)
        # max_movement = abs(target_pose[joint] - self.cur_pose[joint] for joint in target_pose)
        q_movement = abs(q_des - self.joint_state.position)

        duration = max(max(q_movement / self.fr3_max_dq), 0.5)
        point.time_from_start = rospy.Duration.from_sec(duration)
        # the robot should reach the specified joint positions (and other parameters) 00 seconds after the trajectory execution begins.


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

    def set_pose_direct(self, Tb_ed):
        q_des = panda.ik(Tb_ed, q0=self.joint_state.position)
        q_distance = np.abs(q_des - self.joint_state.position)
        if np.all(q_distance) < np.deg2rad(10):
            self.set_joint_direct(q_des)
        else:
            self.set_joint(q_des)


    def set_cartesian_pose(self, Tb_ed, Tb_ee):
        if Tb_ee is None:
            Tb_ee = panda.fk(self.joint_state.position)
        q_des = panda.ik(Tb_ed, q0=self.joint_state.position)
        q_distance = np.abs(q_des - self.joint_state.position)
        duration = max(q_distance / self.fr3_max_dq)
        Ts = interpolate_T(start_T=Tb_ee, end_T=Tb_ed, duration=duration)
        for T in Ts:
            q_des = panda.ik(T, q0=self.joint_state.position)
            self.set_joint_direct(q_des=q_des)





if __name__ == "__main__":
    pjtc = PJTC()
    cur_q = pjtc.joint_state.position
    targ_q = cur_q + 0.05 * np.ones(7)
    print(cur_q)
    print(targ_q)
    # pjtc.set_joint_direct(q_des=targ_q)
    pjtc.set_joint(q_des=targ_q)
    # import time
    # time.sleep(5)
    # exit()

    # cur_T = panda.fk(cur_q)[0][-1]
    # targ_T = np.eye(4)
    # targ_T[:3, -1] = cur_T[:3, -1] + np.array([0.05, 0.05, 0.05])
    # print(cur_T)
    # print(targ_T)
    # pjtc.set_cartesian_pose(Tb_ed=targ_T, Tb_ee=cur_T)

