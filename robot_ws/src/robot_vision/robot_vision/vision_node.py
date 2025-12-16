import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import cv2
import numpy as np

# Import custom messages
from robot_msgs.msg import DetectedShape, DetectedShapes
from geometry_msgs.msg import Point2D, Point3D
from std_msgs.msg import Header

# Import YOLOv8
# It's assumed that ultralytics is installed and the model is available.
# For simulation/testing, a dummy model or a pre-trained COCO model can be used.
try:
    from ultralytics import YOLO
    # Path to your custom-trained YOLOv8 model (e.g., in a 'models' directory within the package)
    # model_path = "path/to/your/best.pt"
    # If using a pre-trained model for testing, uncomment below, e.g., 'yolov8n.pt'
    # yolov8_model = YOLO('yolov8n.pt')
    print("YOLOv8 library imported successfully.")
except ImportError:
    yolov8_model = None
    print("YOLOv8 (ultralytics) not found. Vision node will run in dummy mode.")

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        # Declare parameters for model path and confidence threshold
        self.declare_parameter('yolov8_model_path', 'yolov8n.pt') # Default to a small pre-trained model
        self.declare_parameter('confidence_threshold', 0.5)

        self.yolov8_model_path = self.get_parameter('yolov8_model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        self.get_logger().info(f"Loading YOLOv8 model from: {self.yolov8_model_path}")
        if yolov8_model is None: # In case ultralytics wasn't imported
            self.get_logger().warn("YOLOv8 model could not be loaded due to missing ultralytics library.")
            self.model = None # Indicate no model is loaded
        else:
            try:
                # Attempt to load the user-specified model, or a default if none specified/found
                # For a real project, model_path should point to a .pt file
                self.model = YOLO(self.yolov8_model_path)
                self.get_logger().info("YOLOv8 model loaded successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
                self.model = None

        # Image subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription # prevent unused variable warning

        # Detected shapes publisher
        self.publisher_ = self.create_publisher(DetectedShapes, '/detected_shapes', 10)
        self.get_logger().info('Vision Node initialized and ready to detect shapes.')

        # Placeholder for class names based on training. Example:
        self.class_names = ['circle', 'triangle', 'rectangle', 'square', 'container_circle', 'container_triangle', 'container_rectangle', 'container_square']

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        detected_shapes_msg = DetectedShapes()
        detected_shapes_msg.header = msg.header # Preserve timestamp and frame_id

        if self.model:
            try:
                # Perform inference
                results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)

                # Process results
                for r in results:
                    # Iterate over detected bounding boxes and their attributes
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])

                        if class_id < len(self.class_names):
                            class_label = self.class_names[class_id]
                        else:
                            class_label = f"unknown_{class_id}"
                            self.get_logger().warn(f"Unknown class ID: {class_id} detected.")
                            continue # Skip unknown classes

                        # Populate custom DetectedShape message
                        detected_shape = DetectedShape()
                        detected_shape.class_label = class_label
                        detected_shape.confidence = confidence

                        # Pixel coordinates
                        detected_shape.pixel_center.x = float((x1 + x2) / 2)
                        detected_shape.pixel_center.y = float((y1 + y2) / 2)
                        detected_shape.width = float(x2 - x1)
                        detected_shape.height = float(y2 - y1)

                        # Placeholder for world_coords (requires camera calibration/depth data)
                        # For now, initialize to 0.0
                        detected_shape.world_coords = Point3D(x=0.0, y=0.0, z=0.0)

                        detected_shapes_msg.shapes.append(detected_shape)

                self.publisher_.publish(detected_shapes_msg)
                if len(detected_shapes_msg.shapes) > 0:
                    self.get_logger().info(f"Published {len(detected_shapes_msg.shapes)} detected shapes.")

            except Exception as e:
                self.get_logger().error(f"Error during YOLOv8 inference: {e}")
        else:
            # Dummy detection if model not loaded
            self.get_logger().warn("No YOLOv8 model loaded. Performing dummy detection...")
            dummy_shape = DetectedShape()
            dummy_shape.class_label = "dummy_square"
            dummy_shape.confidence = 0.99
            dummy_shape.pixel_center.x = 320.0
            dummy_shape.pixel_center.y = 240.0
            dummy_shape.width = 50.0
            dummy_shape.height = 50.0
            dummy_shape.world_coords = Point3D(x=1.0, y=0.0, z=0.825)
            detected_shapes_msg.shapes.append(dummy_shape)
            self.publisher_.publish(detected_shapes_msg)
            self.get_logger().info("Published dummy detected shape.")

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
