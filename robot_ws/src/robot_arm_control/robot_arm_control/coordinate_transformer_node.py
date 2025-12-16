import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from cv_bridge import CvBridge
import numpy as np
import yaml
import os

# Import custom messages
from robot_msgs.msg import DetectedShape, DetectedShapes
from geometry_msgs.msg import Point2D, Point3D

# For camera model
from image_geometry import PinholeCameraModel

class CoordinateTransformerNode(Node):

    def __init__(self):
        super().__init__('coordinate_transformer_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()

        # Declare parameter for camera info file path
        self.declare_parameter('camera_info_file', 'camera_info.yaml')
        camera_info_file_name = self.get_parameter('camera_info_file').get_parameter_value().string_value

        # Resolve full path to camera_info.yaml (assuming it's in a config dir within the package)
        # This assumes camera_info.yaml will be installed to share/robot_arm_control/config
        package_share_directory = os.path.join(os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0], 'share', 'robot_arm_control')
        camera_info_path = os.path.join(package_share_directory, 'config', camera_info_file_name)

        if os.path.exists(camera_info_path):
            self.get_logger().info(f"Loading camera info from: {camera_info_path}")
            with open(camera_info_path, 'r') as file:
                camera_info_dict = yaml.safe_load(file)
            self.camera_info = self._dict_to_camera_info_msg(camera_info_dict)
            self.camera_model.fromCameraInfo(self.camera_info)
            self.get_logger().info("Camera intrinsics loaded successfully.")
        else:
            self.get_logger().error(f"Camera info file not found at {camera_info_path}. Cannot perform 2D to 3D conversion.")
            self.camera_info = None

        # Subscribe to detected shapes from vision node
        self.subscription = self.create_subscription(
            DetectedShapes,
            '/detected_shapes',
            self.detected_shapes_callback,
            10)
        self.subscription # prevent unused variable warning

        # Publisher for 3D world coordinates
        self.publisher_ = self.create_publisher(DetectedShapes, '/detected_shapes_3d_world', 10)
        self.get_logger().info('Coordinate Transformer Node initialized.')

    def _dict_to_camera_info_msg(self, data):
        msg = CameraInfo()
        # Assuming only K, D, R, P are needed, adjust if full header/etc. is desired
        msg.height = data.get('height')
        msg.width = data.get('width')
        msg.distortion_model = data.get('distortion_model')
        msg.d = data.get('D')
        msg.k = data.get('K')
        msg.r = data.get('R')
        msg.p = data.get('P')
        return msg

    def detected_shapes_callback(self, msg):
        if not self.camera_info or not self.camera_model.tfFrame():
            self.get_logger().warn("Camera info or camera model not properly initialized. Skipping 2D to 3D conversion.")
            return

        transformed_shapes_msg = DetectedShapes()
        transformed_shapes_msg.header = msg.header

        for detected_shape in msg.shapes:
            # Convert pixel_center to 3D ray in camera frame
            # pixel_u, pixel_v = detected_shape.pixel_center.x, detected_shape.pixel_center.y

            # Example for unprojecting a point. Needs depth to get actual 3D point.
            # If only 2D pixel is available, unproject_pixel creates a ray.
            # For pick and place, we usually need the Z coordinate (depth).
            # This placeholder assumes an average depth or uses `unproject_pixel` for a ray.
            # A proper solution would involve a depth camera or stereo vision.
            # For now, let's assume a known Z for simplicity of demonstration, or use unproject_pixel for ray.

            # Option 1: Use unproject_pixel to get a ray direction vector in camera frame
            # ray = self.camera_model.unproject_pixel(
            #     (detected_shape.pixel_center.x, detected_shape.pixel_center.y))
            # # To get a point, you need depth. Let's assume a fixed Z for now for objects on table.
            # # For example, if object Z is at table_height above camera frame origin.
            # # This is a simplification. Actual depth would come from a depth sensor.
            # estimated_depth = 0.7 # Example depth in meters from camera
            # point_in_camera_frame = PointStamped()
            # point_in_camera_frame.header.frame_id = self.camera_info.header.frame_id # Should be camera_link
            # point_in_camera_frame.header.stamp = msg.header.stamp
            # point_in_camera_frame.point.x = ray[0] * estimated_depth
            # point_in_camera_frame.point.y = ray[1] * estimated_depth
            # point_in_camera_frame.point.z = estimated_depth

            # Option 2: Simplified 3D point using inverse projection with a known Z-depth
            # This is a common simplification for tabletop tasks if actual depth is not used from camera
            # We need the full projection matrix for this, P[0,0] = fx, P[1,1]=fy, P[0,2]=cx, P[1,2]=cy
            # And inverse Z from camera to table for object
            fx = self.camera_info.p[0]
            fy = self.camera_info.p[5]
            cx = self.camera_info.p[2]
            cy = self.camera_info.p[6]

            # Let's assume a constant object height from camera's perspective for now
            # This should ideally come from a depth sensor or 3D reconstruction.
            object_z_in_camera_frame = 0.7 # meters, example depth from camera to object

            # Inverse projection to get X, Y in camera frame
            # X_c = (u - cx) * Z_c / fx
            # Y_c = (v - cy) * Z_c / fy
            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header.frame_id = self.camera_info.header.frame_id or 'camera_link' # Ensure frame_id is set
            point_in_camera_frame.header.stamp = msg.header.stamp

            point_in_camera_frame.point.x = (detected_shape.pixel_center.x - cx) * object_z_in_camera_frame / fx
            point_in_camera_frame.point.y = (detected_shape.pixel_center.y - cy) * object_z_in_camera_frame / fy
            point_in_camera_frame.point.z = object_z_in_camera_frame

            # Lookup transform from camera frame to base_link
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', # Target frame
                    point_in_camera_frame.header.frame_id, # Source frame
                    point_in_camera_frame.header.stamp, # Time at which to get the transform
                    rclpy.duration.Duration(seconds=0.1) # Timeout
                )

                # Transform the point
                point_in_base_link = self.tf_buffer.transform(point_in_camera_frame, 'base_link')

                # Update the DetectedShape message with world_coords
                new_detected_shape = DetectedShape()
                new_detected_shape.class_label = detected_shape.class_label
                new_detected_shape.confidence = detected_shape.confidence
                new_detected_shape.pixel_center = detected_shape.pixel_center
                new_detected_shape.width = detected_shape.width
                new_detected_shape.height = detected_shape.height
                new_detected_shape.world_coords.x = point_in_base_link.point.x
                new_detected_shape.world_coords.y = point_in_base_link.point.y
                new_detected_shape.world_coords.z = point_in_base_link.point.z

                transformed_shapes_msg.shapes.append(new_detected_shape)
                self.get_logger().info(f"Transformed {detected_shape.class_label} to world coords: "
                                       f"x={point_in_base_link.point.x:.2f}, y={point_in_base_link.point.y:.2f}, z={point_in_base_link.point.z:.2f}")

            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().error(f"Could not transform from {point_in_camera_frame.header.frame_id} to base_link: {ex}")
                # If transform fails, publish original shape with default world_coords (0,0,0)
                transformed_shapes_msg.shapes.append(detected_shape)
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred during TF transformation: {e}")
                transformed_shapes_msg.shapes.append(detected_shape)

        if transformed_shapes_msg.shapes:
            self.publisher_.publish(transformed_shapes_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
