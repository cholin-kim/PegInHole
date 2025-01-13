import rospy
from controller_manager_msgs.srv import (SwitchController, SwitchControllerRequest, LoadController, LoadControllerRequest,
                                         UnloadController, UnloadControllerRequest, ListControllers, ListControllerTypes, ListControllersRequest)
# from controller_manager_msgs.msg import ControllerState


class Controller_Switch:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('controller_switcher')

        # List of all used controllers
        # self.fs_str = "franka_state_controller"
        # self.cic_str = "cartesian_impedance_example_controller"
        # self.pjtc_str = "position_joint_trajectory_controller"
        # self.ejtc_str = "effort_joint_trajectory_controller"


        rospy.wait_for_service('/controller_manager/switch_controller')
        rospy.wait_for_service('/controller_manager/load_controller')
        rospy.wait_for_service('/controller_manager/list_controllers')
        rospy.wait_for_service('/controller_manager/unload_controller')

        load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        self.list_controller = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        self._switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)



        self.current_controller = None
        self.find_current_controller()

        # First, load all used controllers
        load_controller('arm_position_controller')
        load_controller('teleop_shared_controller')

        self.req = SwitchControllerRequest()
        self.req.strictness = SwitchControllerRequest.STRICT
        self.req.start_asap = True
        self.req.timeout = 1.0

        # self.start_controller(des_ctrl)
        # self.switch_controller(des_ctrl)

    def find_current_controller(self):
        response = self.list_controller()
        self.current_controller = [controller.name for controller in response.controller if controller.state == 'running']


    def start_controller(self, des_ctrl):
        self.req.start_controllers = [des_ctrl]
        self.req.stop_controllers = []
        try:
            result = self._switch_controller(self.req)
            if result.ok:
                print('done')
            else:
                print('failed')
        except rospy.ServiceException as e:
            print(e)

    def switch_controller(self, des_ctrl):
        if des_ctrl == self.current_controller[-1]:
            print(f'The controller named {des_ctrl} is already activated.')
        else:
            self.req.start_controllers = [des_ctrl]
            self.req.stop_controllers = [item for item in self.current_controller if not item.startswith('f')]
            try:
                result = self._switch_controller(self.req)
                if result.ok:
                    print('done')
                else:
                    print('failed')
            except rospy.ServiceException as e:
                print(e)





if __name__ == "__main__":
    sc = Controller_Switch()
    print("current activate controllers:", sc.current_controller)
    # sc.switch_controller(des_ctrl='teleop_shared_controller')
    sc.switch_controller(des_ctrl='arm_position_controller')

    # Unload
    # A controller can only be unloaded when it is in the stopped state.
