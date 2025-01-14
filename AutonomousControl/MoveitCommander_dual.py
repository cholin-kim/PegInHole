import sys
import copy

import franka_msgs.msg
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
from Kinematics.panda.pandaKinematics import pandaKinematics
from Kinematics.panda.pandaVar import gripper_len
panda = pandaKinematics()



## currently cannot executable via moveit, since controller not connected.
## executing with other controller

class MvitCommander_dual:
    def __init__(self, group_name):
        print("============ Starting tutorial setup")
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.get_node_uri():
            rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        self.group_name = group_name

        self.robot = moveit_commander.RobotCommander("/combined_panda/robot_description", ns="/combined_panda")
        self.group = moveit_commander.MoveGroupCommander(name=self.group_name, robot_description="/combined_panda/robot_description", ns="/combined_panda")

        self.group.set_max_velocity_scaling_factor(0.7)
        self.group.set_max_acceleration_scaling_factor(0.5)
        # self.display_trajectory_publisher = rospy.Publisher(self.group_name + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        ## Getting Basic Information
        #
        # if self.group_name == "gripper_arm":
        #     self.group.set_pose_reference_frame("panda_2_link0")
        # elif self.group_name == "camera_arm":
        #     self.group.set_pose_reference_frame("panda_1_link0")
        print("============ Reference frame: %s" % self.group.get_planning_frame())
        print("============ EE link: %s" % self.group.get_end_effector_link())
        print("============ Robot Groups:")
        print(self.robot.get_group_names())


        self.joint_state = self.group.get_current_joint_values()
        # print("============ Printing robot state")
        # print( self.robot.get_current_state())
        # print(self.robot.get_joint_names())
        # print( "============")


    def set_joint(self, targ_q, execute=False):
        ## Warning
        # joint values should be obtained after considering gripper len!!!
        ##

        self.group.clear_pose_targets()

        group_variable_values = self.group.get_current_joint_values()
        print("============ Joint values: ", group_variable_values)
        group_variable_values = targ_q
        self.group.set_joint_value_target(group_variable_values)

        plan = self.group.plan()
        rospy.sleep(2)

        # Done automatically by group.plan()
        # print "============ Visualizing plan1"
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory_start = self.robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # self.display_trajectory_publisher.publish(display_trajectory);
        # print ("============ Waiting while plan1 is visualized (again)...")
        # rospy.sleep(5)

        if execute: self.group.go(wait=True)

        result, joint_traj = plan[0], plan[1].joint_trajectory.points  # Bool, list
        return result, joint_traj


### Currently unable to plan trajectory in local frame. Remapping T from base to panda_{i}_link0.###


    def set_Tb_ed(self, Tb_ed, execute=False):
        self.group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        Tb_ed = self.preprocess_T(Tb_ed=Tb_ed)

        target_ori = R.from_matrix(Tb_ed[:3, :3]).as_quat()
        pose_target.orientation.x = target_ori[0]
        pose_target.orientation.y = target_ori[1]
        pose_target.orientation.z = target_ori[2]
        pose_target.orientation.w = target_ori[3]
        pose_target.position.x = Tb_ed[0, -1]
        pose_target.position.y = Tb_ed[1, -1]
        pose_target.position.z = Tb_ed[2, -1]
        self.group.set_pose_target(pose_target)

        plan = self.group.plan()
        # print(plan)
        rospy.sleep(2)

        if execute: self.group.go(wait=True)


    def set_cartesian_path(self, Tb_ed, execute=False):
        self.group.clear_pose_targets()
        waypoints = []
        Tb_ed = self.preprocess_T(Tb_ed=Tb_ed)

        # start with current pose <-- do not add current pose, this will interrupt duration.
        # waypoints.append(self.group.get_current_pose().pose)

        wpose = geometry_msgs.msg.Pose()
        target_ori = R.from_matrix(Tb_ed[:3, :3]).as_quat()
        wpose.orientation.x = target_ori[0]
        wpose.orientation.y = target_ori[1]
        wpose.orientation.z = target_ori[2]
        wpose.orientation.w = target_ori[3]
        wpose.position.x = Tb_ed[0, -1]
        wpose.position.y = Tb_ed[1, -1]
        wpose.position.z = Tb_ed[2, -1]
        waypoints.append(copy.deepcopy(wpose))

        # print(waypoints)
        planning_step = 0.001  # 0.01 = plan eef with 1cm step, lower than 1mm makes robot unstable
        (plan, fraction) = self.group.compute_cartesian_path(waypoints=waypoints, eef_step=planning_step)

        #########################################################################################
        ## !!!Do not modify duration!!! ##
        # duration = np.linalg.norm([(waypoints[0].position.x - waypoints[-1].position.x), (waypoints[0].position.y - waypoints[-1].position.y), (waypoints[0].position.z - waypoints[-1].position.z)]) / planning_step
        # t = np.linspace(0, duration, len(plan.joint_trajectory.points))
        # for i in range(len(plan.joint_trajectory.points)):
        #     plan.joint_trajectory.points[i].time_from_start = rospy.Duration.from_sec(t[i])
        #########################################################################################

        # print(plan)

        # rospy.sleep(2)
        # if execute: self.group.execute(plan, wait=True)




    def set_pose(self, pose, execute=False):
        '''
        :param pose: geometry_msgs.msg.Pose()
        '''
        pose_target = pose
        self.group.set_pose_target(pose_target)

        plan = self.group.plan()
        rospy.sleep(2)

        # if execute: self.group.go(wait=True)

    def preprocess_T(self, Tb_ed):
        Tflagne_tcp = np.identity(4)
        Tflagne_tcp[2, -1] = gripper_len
        Tb_flange = Tb_ed @ np.linalg.inv(Tflagne_tcp)
        return Tb_flange


if __name__ == "__main__":
    mvit_gripper = MvitCommander_dual(group_name="pd1")
    mvit_camera = MvitCommander_dual(group_name="pd2")

    cur_q_gripper = mvit_gripper.joint_state
    cur_q_camera = mvit_camera.joint_state
    print("cur_q_gripper:", cur_q_gripper)
    print("cur_q_camera:", cur_q_camera)


    ## 1. Joint command
    target_q_gripper = copy.deepcopy(cur_q_gripper)
    target_q_gripper += -0.05 * np.ones(7)
    target_q_camera = copy.deepcopy(cur_q_camera)
    target_q_camera += -0.05 * np.ones(7)
    # mvit_gripper.set_joint(targ_q=target_q_gripper)
    # mvit_camera.set_joint(targ_q=target_q_camera)


    ## 2. Tb_ed command
    # Tb_ed_gripper = panda.fk(cur_q_gripper)[0][-1]
    message_camera = rospy.wait_for_message("/combined_panda/panda_2_state_controller/franka_states", franka_msgs.msg.FrankaState)
    Tb_ee_camera = np.array(message_camera.O_T_EE).reshape(4, 4).T
    # Tb_ed[:3, :3] = R.from_euler('XZ', [np.pi, -np.pi/2]).as_matrix()
    Tb_ee_camera[:3, -1] += [0.0, 0.0, 0.01]

        # 2-1. Joint space
    mvit_camera.set_Tb_ed(Tb_ed=Tb_ee_camera, execute=False)
    exit()
        # 2-2. Cartesian space
    # mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=False)
    # mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=True)

    ## 3. Pose command(joint space)
    # pose_target = geometry_msgs.msg.Pose()
    # target_ori = R.from_matrix(Tb_ed[:3, :3]).as_quat()
    # pose_target.orientation.x = target_ori[0]
    # pose_target.orientation.y = target_ori[1]
    # pose_target.orientation.z = target_ori[2]
    # pose_target.orientation.w = target_ori[3]
    # pose_target.position.x = Tb_ed[0, -1]
    # pose_target.position.y = Tb_ed[1, -1]
    # pose_target.position.z = Tb_ed[2, -1]
    # mvit.set_pose(pose=pose_target, execute=True)

