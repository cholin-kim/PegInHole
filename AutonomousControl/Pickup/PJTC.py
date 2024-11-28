from turtledemo.forest import start

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
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline

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
        q_traj = self.interpolate_q(start_q=self.joint_state.position, end_q=q_des, duration=duration)
        for q in q_traj:
            self.set_joint_direct(targ_q=q)
            # 아니면 point를 append해서 한번에 보내야 할까?


    def set_joint_direct(self, q_des=None):
        '''
        :param target_q(list)
        '''

        target_pose = copy.deepcopy(self.joint_state.position)
        if q_des is None:
            for key in target_pose:
                target_pose[key] += float(0.05 * np.random.randint(-1, 2))

        else:
            i =0
            for key in target_pose:
                target_pose[key] = q_des[i]
                i += 1

        print("current_pose:", self.joint_state.position)
        print("target_pose:", q_des)

        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()


        goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*target_pose.items())]
        point.velocities = [0] * len(target_pose)

        goal.trajectory.points.append(point)
        goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)


        # max_movement = max(abs(target_pose[joint] - self.cur_pose[joint]) for joint in target_pose)
        max_movement = abs(target_pose[joint] - self.cur_pose[joint] for joint in target_pose)

        point.time_from_start = rospy.Duration.from_sec(max(max_movement / self.fr3_max_dq, 0.5))
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

    def set_pose(self, Tb_ed, Tb_ee=None):
        q_des = panda.ik(Tb_ed, q0=self.joint_state.position)
        q_distance = np.abs(q_des - self.joint_state.position)
        if Tb_ee is None:
            # 작은 움직임이되, target만 cartesian으로 준것으로 생각.
            # check 작은 움직임.
            # cur_T = panda.fk(self.joint_state.position)[0][-1]
            # pos_distance = np.any(np.abs(cur_T[:3, -1] - targ_T[:3, -1]) > 0.03)
            # ori_distance = np.any()
            if np.all(q_distance) < np.deg2rad(10):
                self.set_joint_direct(q_des)
            else:
                self.set_joint(q_des)
        else:
            # cartesian linear interpolation을 원한다고 생각.
            # interpolate linearly using the similar duration factor. but this time, cartesian.
            # compute each q for each target T, then send goal with self.set_joint_drect(q)
            duration = max(q_distance / self.fr3_max_dq)
            Ts = self.interpolate_T(start_T=Tb_ee, end_T=Tb_ed, duration=duration)
            for T in Ts:
                q_des = panda.ik(T, q0=self.joint_state.position)
                self.set_joint_direct(q_des=q_des)




    def interpolate_q(self, start_q, end_q, duration):
        t = np.array([0, duration])
        cs = CubicSpline(t, np.array([start_q, end_q]), bc_type='natural')

        ts = np.linspace(0, duration, int(duration * 50))
        q_traj = cs(ts)
        return q_traj

    def interpolate_T(self, start_T, end_T, duration):
        t = np.array([0, duration])
        pos = start_T[:3, -1]
        pos_des = end_T[:3, -1]

        cs = CubicSpline(t, np.array([pos, pos_des]), bc_type='natual')

        ori = R.from_matrix(start_T[:3, :3])
        ori_des = R.from_matrix(end_T[:3, :3])

        RS = RotationSpline(t, R.concatenate([ori, ori_des]))

        ts = np.linspace(0, duration, int(duration * 100))
        pos_traj = cs(ts)
        ori_traj = RS(ts).as_matrix()
        Ts = np.zeros((len(pos_traj), 4, 4))
        Ts[:, -1, -1] = 1
        Ts[:, :3, -1] = pos_traj
        Ts[:, :3, :3] = ori_traj
        return Ts




if __name__ == "__main__":
    pjtc = PJTC()
    while True:
        pjtc.move_to_target_direct()
