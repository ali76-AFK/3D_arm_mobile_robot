import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.timer import Timer
from enum import Enum

# ROS2 Messages and Services
from robot_msgs.msg import DetectedShapes, DetectedShape
from robot_msgs.srv import PickObject, PlaceObject, StartTask
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class TaskState(Enum):
    IDLE = 0
    NAVIGATING_TO_PICK = 1
    SCANNING_FOR_SHAPES = 2
    PICKING_OBJECT = 3
    NAVIGATING_TO_PLACE = 4
    SCANNING_FOR_CONTAINERS = 5
    PLACING_OBJECT = 6
    TASK_COMPLETE = 7
    ERROR = 8

class TaskOrchestrationNode(Node):

    def __init__(self):
        super().__init__('task_orchestration_node')
        self.get_logger().info('Task Orchestration Node initialized.')

        self.current_state = TaskState.IDLE
        self.detected_shapes = []
        self.current_target_shape = None
        self.current_target_container_label = None

        # --- ROS2 Clients and Subscriptions ---
        self.shape_subscription = self.create_subscription(
            DetectedShapes,
            '/detected_shapes_3d_world',
            self.detected_shapes_callback,
            10
        )

        self.pick_client = self.create_client(PickObject, 'pick_object')
        self.place_client = self.create_client(PlaceObject, 'place_object')
        self.start_task_service = self.create_service(StartTask, 'start_task', self.start_task_callback)

        # Nav2 Action Client
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_action_client.wait_for_server()

        self.timer = self.create_timer(1.0, self.state_machine_loop) # Loop at 1 Hz

        self.get_logger().info(f'Current state: {self.current_state.name}')

    def start_task_callback(self, request, response):
        self.get_logger().info('Received start task request.')
        if self.current_state == TaskState.IDLE or self.current_state == TaskState.TASK_COMPLETE:
            self.current_state = TaskState.NAVIGATING_TO_PICK
            self.get_logger().info(f'Transitioned to state: {self.current_state.name}')
            response.success = True
            response.message = "Task started."
        else:
            response.success = False
            response.message = f"Task already in progress ({self.current_state.name})."
        return response

    def detected_shapes_callback(self, msg):
        if self.current_state == TaskState.SCANNING_FOR_SHAPES or            self.current_state == TaskState.SCANNING_FOR_CONTAINERS:
            self.detected_shapes = msg.shapes
            self.get_logger().debug(f'Received {len(msg.shapes)} detected shapes/containers.')

    def state_machine_loop(self):
        if self.current_state == TaskState.IDLE:
            # Wait for start task command
            pass

        elif self.current_state == TaskState.NAVIGATING_TO_PICK:
            self.get_logger().info('Attempting to navigate to pick table.')
            pick_table_pose = self.get_pick_table_pose() # Implement this function
            self.send_nav_goal(pick_table_pose, self.nav_to_pick_callback)
            self.current_state = TaskState.SCANNING_FOR_SHAPES # Optimistically transition

        elif self.current_state == TaskState.SCANNING_FOR_SHAPES:
            if self.detected_shapes:
                self.get_logger().info(f'Found {len(self.detected_shapes)} shapes. Selecting one...')
                # Implement logic to select the best shape to pick
                self.current_target_shape = self.select_shape_to_pick(self.detected_shapes)
                if self.current_target_shape:
                    self.current_state = TaskState.PICKING_OBJECT
                    self.get_logger().info(f'Selected {self.current_target_shape.class_label} for picking.')
                else:
                    self.get_logger().warn('No suitable shape found. Retrying scan.')
                    # Error handling: maybe re-scan or move slightly
            else:
                self.get_logger().warn('No shapes detected. Ensure camera is active and objects are present.')
                # Error handling: maybe move camera or wait

        elif self.current_state == TaskState.PICKING_OBJECT:
            if self.current_target_shape:
                self.get_logger().info(f'Calling pick service for {self.current_target_shape.class_label}.')
                request = PickObject.Request()
                request.target_object = self.current_target_shape
                self.future_pick = self.pick_client.call_async(request)
                self.future_pick.add_done_callback(self.pick_response_callback)
                self.current_state = TaskState.IDLE # Wait for response
            else:
                self.get_logger().error('No target shape defined for picking. Transitioning to ERROR.')
                self.current_state = TaskState.ERROR

        elif self.current_state == TaskState.NAVIGATING_TO_PLACE:
            self.get_logger().info('Attempting to navigate to place table.')
            place_table_pose = self.get_place_table_pose() # Implement this function
            self.send_nav_goal(place_table_pose, self.nav_to_place_callback)
            self.current_state = TaskState.SCANNING_FOR_CONTAINERS # Optimistically transition

        elif self.current_state == TaskState.SCANNING_FOR_CONTAINERS:
            if self.detected_shapes:
                self.get_logger().info(f'Found {len(self.detected_shapes)} containers. Selecting one...')
                # Implement logic to select the correct container for the picked shape
                self.current_target_container_label, place_pose = self.select_container_for_shape(self.detected_shapes, self.current_target_shape)
                if self.current_target_container_label and place_pose:
                    self.place_pose = place_pose
                    self.current_state = TaskState.PLACING_OBJECT
                    self.get_logger().info(f'Selected {self.current_target_container_label} for placing.')
                else:
                    self.get_logger().warn('No suitable container found. Retrying scan.')
            else:
                self.get_logger().warn('No containers detected. Retrying scan.')

        elif self.current_state == TaskState.PLACING_OBJECT:
            if self.current_target_container_label and self.place_pose:
                self.get_logger().info(f'Calling place service for {self.current_target_container_label}.')
                request = PlaceObject.Request()
                request.container_label = self.current_target_container_label
                request.target_pose = self.place_pose
                self.future_place = self.place_client.call_async(request)
                self.future_place.add_done_callback(self.place_response_callback)
                self.current_state = TaskState.IDLE # Wait for response
            else:
                self.get_logger().error('No target container or pose defined for placing. Transitioning to ERROR.')
                self.current_state = TaskState.ERROR

        elif self.current_state == TaskState.TASK_COMPLETE:
            self.get_logger().info('Task completed successfully.')
            # Reset for next task
            self.current_state = TaskState.IDLE

        elif self.current_state == TaskState.ERROR:
            self.get_logger().error('An error occurred. Attempting to recover or restart.')
            # Implement error recovery logic here (e.g., re-attempt, go to safe pose, notify user)
            self.current_state = TaskState.IDLE # For now, reset to IDLE

    # --- Helper Functions for Navigation ---
    def send_nav_goal(self, pose, callback_func):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._nav_action_client.send_goal_async(goal_msg).add_done_callback(lambda future: self.nav_goal_response(future, callback_func))

    def nav_goal_response(self, future, callback_func):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected.')
            self.current_state = TaskState.ERROR
        else:
            self.get_logger().info('Navigation goal accepted.')
            goal_handle.get_result_async().add_done_callback(callback_func)

    def nav_to_pick_callback(self, future):
        result = future.result().result
        if result.error_code == NavigateToPose.Result.NONE:
            self.get_logger().info('Successfully navigated to pick table.')
            self.current_state = TaskState.SCANNING_FOR_SHAPES
        else:
            self.get_logger().error(f'Navigation to pick table failed with error code: {result.error_code}.')
            self.current_state = TaskState.ERROR
        self.detected_shapes = [] # Clear previous detections

    def nav_to_place_callback(self, future):
        result = future.result().result
        if result.error_code == NavigateToPose.Result.NONE:
            self.get_logger().info('Successfully navigated to place table.')
            self.current_state = TaskState.SCANNING_FOR_CONTAINERS
        else:
            self.get_logger().error(f'Navigation to place table failed with error code: {result.error_code}.')
            self.current_state = TaskState.ERROR
        self.detected_shapes = [] # Clear previous detections

    # --- Callbacks for Arm/Gripper Services ---
    def pick_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Object picked successfully. Navigating to place.')
                self.current_state = TaskState.NAVIGATING_TO_PLACE
            else:
                self.get_logger().error(f'Pick failed: {response.message}. Transitioning to ERROR.')
                self.current_state = TaskState.ERROR
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.current_state = TaskState.ERROR

    def place_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Object placed successfully. Task complete for this object.')
                self.current_state = TaskState.TASK_COMPLETE # Or go back to scanning for next object
            else:
                self.get_logger().error(f'Place failed: {response.message}. Transitioning to ERROR.')
                self.current_state = TaskState.ERROR
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.current_state = TaskState.ERROR

    # --- Placeholder Logic Functions ---
    def get_pick_table_pose(self) -> PoseStamped:
        # Returns a PoseStamped for the robot to navigate to the pick table
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.5 # Example coordinate
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def get_place_table_pose(self) -> PoseStamped:
        # Returns a PoseStamped for the robot to navigate to the place table
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = -0.5 # Example coordinate
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def select_shape_to_pick(self, shapes: list[DetectedShape]) -> DetectedShape:
        # Example: Pick the shape with the highest confidence
        if not shapes: return None
        best_shape = None
        max_confidence = -1.0
        for shape in shapes:
            if shape.confidence > max_confidence:
                max_confidence = shape.confidence
                best_shape = shape
        return best_shape

    def select_container_for_shape(self, containers: list[DetectedShape], picked_shape: DetectedShape) -> tuple[str, PoseStamped]:
        # Example: Match container label to shape label (e.g., 'circle' -> 'circle_container')
        if not containers or not picked_shape: return None, None
        target_container_label_prefix = picked_shape.class_label
        for container in containers:
            if container.class_label.startswith(target_container_label_prefix):
                # Construct a placement pose above the container
                place_pose = PoseStamped()
                place_pose.header.frame_id = 'map'
                place_pose.header.stamp = self.get_clock().now().to_msg()
                # Placeholder: place object slightly above the container's world_coords
                place_pose.pose.position.x = container.world_coords.x
                place_pose.pose.position.y = container.world_coords.y
                place_pose.pose.position.z = container.world_coords.z + 0.15 # Above container
                place_pose.pose.orientation.w = 1.0 # Default orientation
                return container.class_label, place_pose
        return None, None

def main(args=None):
    rclpy.init(args=args)

    # Ensure the Python package directory exists (for non-ROS environment execution)
    workspace_path = os.path.expanduser('~/robot_ws')
    src_path = os.path.join(workspace_path, 'src')
    robot_task_manager_pkg_path = os.path.join(src_path, 'robot_task_manager')
    python_pkg_dir = os.path.join(robot_task_manager_pkg_path, 'robot_task_manager')
    os.makedirs(python_pkg_dir, exist_ok=True)

    task_orchestration_node = TaskOrchestrationNode()

    # Use a multithreaded executor to handle service calls and action goals concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(task_orchestration_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    task_orchestration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
