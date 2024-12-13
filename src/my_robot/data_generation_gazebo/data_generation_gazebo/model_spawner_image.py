import os

import cv2
import cv_bridge
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node
from sensor_msgs.msg import Image


class ModelSpawner(Node):
    def __init__(self, model_names):
        super().__init__("model_spawner")
        self.model_names = model_names
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")

        # Set GAZEBO_MODEL_PATH to include the package's models directory
        package_name = "data_generation_gazebo"  # Replace with your actual package name
        package_path = get_package_share_directory(package_name)
        self.models_path = os.path.join(package_path, "models")

        # Append to GAZEBO_MODEL_PATH
        if "GAZEBO_MODEL_PATH" in os.environ:
            os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + self.models_path
        else:
            os.environ["GAZEBO_MODEL_PATH"] = self.models_path

        self.get_logger().info(f"GAZEBO_MODEL_PATH set to: {os.environ['GAZEBO_MODEL_PATH']}")

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn_entity service...")

        for model_name in self.model_names:
            self.spawn_model(model_name)

    def spawn_model(self, model_name):
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = self.get_model_sdf(model_name)
        request.robot_namespace = ""
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.0

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Model {model_name} spawned successfully.")
        else:
            self.get_logger().error(f"Failed to spawn model {model_name}.")

    def get_model_sdf(self, model_name):
        # Assuming models are stored in GAZEBO_MODEL_PATH
        model_path = os.path.join(os.environ["GAZEBO_MODEL_PATH"], model_name, "model.sdf")
        with open(model_path, "r") as file:
            return file.read()


class ImageCapture(Node):
    def __init__(self, number_of_images=10):
        super().__init__("image_capture")
        self.image_subscriber = self.create_subscription(
            Image, "/top_rgbd_depth_sensor/image_raw", self.image_callback, number_of_images
        )

        self.bridge = cv_bridge.CvBridge()
        self.image_counter = 0
        self.save_path = "~/data_test/"
        os.makedirs(self.save_path, exist_ok=True)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            filename = os.path.join(self.save_path, f"image_{self.image_counter}.png")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved image: {filename}")
            self.image_counter += 1

        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    model_names = ["book", "biscuit", "coke_can"]  # Replace with your actual model names
    spawner = ModelSpawner(model_names)
    image_capture_node = ImageCapture()
    rclpy.spin(image_capture_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
