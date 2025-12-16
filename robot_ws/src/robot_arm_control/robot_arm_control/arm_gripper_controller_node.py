import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# MoveIt2 Python Interface
import moveit_msgs.msg
import moveit_commander

# Custom service messages
from robot_msgs.srv import PickObject, PlaceObject

class ArmGripperControllerNode(Node):

    def __init__(self):
        super().__init__('arm_gripper_controller_node')
        self.callback_group = ReentrantCallbackGroup()

        self.get_logger().info("Initializing MoveIt2 commander...")
        # Initialize MoveIt2 components
        # This needs to be done *after* the node is initialized
        # and usually in a separate thread or with `rclpy.spin_until_future_complete`
        # For simplicity, we'll initialize them here.
        # Note: 'robot_description' parameter must be available in the ROS parameter server.

        try:
            moveit_commander.roscpp_initialize([])
            self.robot = moveit_commander.RobotCommander(node_name='arm_gripper_controller_node')
            self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)

            self.arm_group = moveit_commander.MoveGroupCommander("arm", robot_commander=self.robot, node_name='arm_gripper_controller_node')
            self.gripper_group = moveit_commander.MoveGroupCommander("gripper", robot_commander=self.robot, node_name='arm_gripper_controller_node')

            self.get_logger().info("MoveIt2 commander initialized successfully.")
            self.get_logger().info(f"Planning frame: {self.arm_group.get_planning_frame()}")
            self.get_logger().info(f"End effector link: {self.arm_group.get_end_effector_link()}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt2 commander: {e}")
            self.robot = None
            self.scene = None
            self.arm_group = None
            self.gripper_group = None

        # Services for pick and place
        self.pick_service = self.create_service(
            PickObject, 'pick_object', self.pick_object_callback, callback_group=self.callback_group)
        self.place_service = self.create_service(
            PlaceObject, 'place_object', self.place_object_callback, callback_group=self.callback_group)

        # Optional: Subscriber for direct pose goals (for testing)
        self.pose_goal_subscriber = self.create_subscription(
            PoseStamped, '/arm_pose_goal', self.arm_pose_goal_callback, 10, callback_group=self.callback_group)

        self.get_logger().info('Arm Gripper Control Node initialized.')

    def pick_object_callback(self, request, response):
        self.get_logger().info(f'Received pick request for: {request.target_object.class_label} at '                                f'world_coords: ({request.target_object.world_coords.x:.2f}, '                                f'{request.target_object.world_coords.y:.2f}, '                                f'{request.target_object.world_coords.z:.2f})')

        if not self.arm_group or not self.gripper_group:
            response.success = False
            response.message = "MoveIt2 groups not initialized."
            self.get_logger().error(response.message)
            return response

        # --- Placeholder for Pick Logic ---
        # 1. Move arm to a pre-grasp pose above the object
        self.get_logger().info("Planning pre-grasp move...")
        pre_grasp_pose = geometry_msgs.msg.Pose()
        pre_grasp_pose.position.x = request.target_object.world_coords.x
        pre_grasp_pose.position.y = request.target_object.world_coords.y
        pre_grasp_pose.position.z = request.target_object.world_coords.z + 0.1 # 10cm above object
        pre_grasp_pose.orientation.w = 1.0 # Pointing straight down (example)

        self.arm_group.set_pose_target(pre_grasp_pose)
        plan_success, plan, _, _ = self.arm_group.plan()

        if plan_success:
            self.get_logger().info("Executing pre-grasp move...")
            self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        else:
            response.success = False
            response.message = "Failed to plan pre-grasp move."
            self.get_logger().error(response.message)
            return response

        # 2. Open gripper
        self.get_logger().info("Opening gripper...")
        self.gripper_group.set_joint_value_target(self.gripper_group.get_named_target_values("open")) # Assumes "open" is a named state
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

        # 3. Move arm to grasp pose (at the object)
        self.get_logger().info("Planning grasp move...")
        grasp_pose = geometry_msgs.msg.Pose()
        grasp_pose.position.x = request.target_object.world_coords.x
        grasp_pose.position.y = request.target_object.world_coords.y
        grasp_pose.position.z = request.target_object.world_coords.z # At object height
        grasp_pose.orientation.w = 1.0

        self.arm_group.set_pose_target(grasp_pose)
        plan_success, plan, _, _ = self.arm_group.plan()

        if plan_success:
            self.get_logger().info("Executing grasp move...")
            self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        else:
            response.success = False
            response.message = "Failed to plan grasp move."
            self.get_logger().error(response.message)
            return response

        # 4. Close gripper
        self.get_logger().info("Closing gripper...")
        self.gripper_group.set_joint_value_target(self.gripper_group.get_named_target_values("closed")) # Assumes "closed" is a named state
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

        # 5. Move arm to a retreat pose (above object with it held)
        self.get_logger().info("Planning retreat move...")
        retreat_pose = geometry_msgs.msg.Pose()
        retreat_pose.position.x = request.target_object.world_coords.x
        retreat_pose.position.y = request.target_object.world_coords.y
        retreat_pose.position.z = request.target_object.world_coords.z + 0.15 # Retreat higher
        retreat_pose.orientation.w = 1.0

        self.arm_group.set_pose_target(retreat_pose)
        plan_success, plan, _, _ = self.arm_group.plan()

        if plan_success:
            self.get_logger().info("Executing retreat move...")
            self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        else:
            response.success = False
            response.message = "Failed to plan retreat move."
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = "Object picked successfully."
        self.get_logger().info(response.message)
        return response

    def place_object_callback(self, request, response):
        self.get_logger().info(f'Received place request for container: {request.container_label}')

        if not self.arm_group or not self.gripper_group:
            response.success = False
            response.message = "MoveIt2 groups not initialized."
            self.get_logger().error(response.message)
            return response

        # --- Placeholder for Place Logic ---
        # Assuming target_pose is provided for precise placement
        place_target_pose = request.target_pose.pose

        # 1. Move arm to a pre-place pose above the container
        self.get_logger().info("Planning pre-place move...")
        pre_place_pose = geometry_msgs.msg.Pose()
        pre_place_pose.position.x = place_target_pose.position.x
        pre_place_pose.position.y = place_target_pose.position.y
        pre_place_pose.position.z = place_target_pose.position.z + 0.1 # 10cm above container
        pre_place_pose.orientation = place_target_pose.orientation

        self.arm_group.set_pose_target(pre_place_pose)
        plan_success, plan, _, _ = self.arm_group.plan()

        if plan_success:
            self.get_logger().info("Executing pre-place move...")
            self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        else:
            response.success = False
            response.message = "Failed to plan pre-place move."
            self.get_logger().error(response.message)
            return response

        # 2. Move arm to place pose (into the container)
        self.get_logger().info("Planning place move...")
        self.arm_group.set_pose_target(place_target_pose)
        plan_success, plan, _, _ = self.arm_group.plan()

        if plan_success:
            self.get_logger().info("Executing place move...")
            self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        else:
            response.success = False
            response.message = "Failed to plan place move."
            self.get_logger().error(response.message)
            return response

        # 3. Open gripper to release object
        self.get_logger().info("Opening gripper to release...")
        self.gripper_group.set_joint_value_target(self.gripper_group.get_named_target_values("open"))
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

        # 4. Move arm to a post-place pose (retreat from container)
        self.get_logger().info("Planning post-place move...")
        post_place_pose = geometry_msgs.msg.Pose()
        post_place_pose.position.x = place_target_pose.position.x
        post_place_pose.position.y = place_target_pose.position.y
        post_place_pose.position.z = place_target_pose.position.z + 0.15 # Retreat higher
        post_place_pose.orientation = place_target_pose.orientation

        self.arm_group.set_pose_target(post_place_pose)
        plan_success, plan, _, _ = self.arm_group.plan()

        if plan_success:
            self.get_logger().info("Executing post-place move...")
            self.arm_group.execute(plan, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
        else:
            response.success = False
            response.message = "Failed to plan post-place move."
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = "Object placed successfully."
        self.get_logger().info(response.message)
        return response

    def arm_pose_goal_callback(self, msg):
        self.get_logger().info(f"Received direct arm pose goal: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}")
        if self.arm_group:
            self.arm_group.set_pose_target(msg.pose)
            plan_success, plan, _, _ = self.arm_group.plan()
            if plan_success:
                self.get_logger().info("Executing arm pose goal...")
                self.arm_group.execute(plan, wait=True)
                self.arm_group.stop()
                self.arm_group.clear_pose_targets()
                self.get_logger().info("Arm pose goal executed.")
            else:
                self.get_logger().warn("Failed to plan arm pose goal.")
        else:
            self.get_logger().warn("Arm MoveGroupCommander not initialized.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmGripperControllerNode()

    # Use a MultiThreadedExecutor to allow services and subscriptions to run concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    moveit_commander.roscpp_shutdown() # Shutdown MoveIt2 commander

if __name__ == '__main__':
    main()
