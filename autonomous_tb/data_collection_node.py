import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.bridge = CvBridge()
        self.image_count = 0
        self.lidar_count = 0

        self.image_save_path = 'src/autonomous_tb/sim_data/images'
        self.lidar_save_path = 'src/autonomous_tb/sim_data/lidar'

        os.makedirs(self.image_save_path, exist_ok=True)
        os.makedirs(self.lidar_save_path, exist_ok=True)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_filename = os.path.join(self.image_save_path, f'image_{self.image_count:04d}.jpg')
        cv2.imwrite(image_filename, cv_image)
        self.image_count += 1
        self.get_logger().info(f'Saved {image_filename}')

    def lidar_callback(self, msg):
        lidar_data = np.array(msg.ranges)
        lidar_filename = os.path.join(self.lidar_save_path, f'lidar_{self.lidar_count:04d}.npy')
        np.save(lidar_filename, lidar_data)
        self.lidar_count += 1
        self.get_logger().info(f'Saved {lidar_filename}')

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()