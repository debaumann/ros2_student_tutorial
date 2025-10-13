import numpy as np 
import rclpy 
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayLayout, MultiArrayDimension
import cv2
import os

class Roller(Node):
    def __init__(self):
        super().__init__('roller')
        self.publisher = self.create_publisher(UInt8MultiArray, 'image_array', 10)
        self.parameters = self.declare_parameter('data_path', 'talker_listener/data')
        rclpy.logging.get_logger('roller').info(f'Parameter value: {self.parameters.value}')
        self.relative_path = self.parameters.value
        self.image_paths = sorted([img for img in os.listdir(self.relative_path) if img.endswith(('.png', '.jpg', '.jpeg'))])
        self.images = self.load_images()
        self.index = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
    def load_images(self):
        images = []
        for f in self.image_paths:
            img_path = os.path.join(self.relative_path, f)
            cv_image = cv2.imread(img_path)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv_image = cv2.resize(cv_image, (200, 160))
            self.img_shape = list(cv_image.shape)
            images.append(cv_image.flatten().tolist())
        return images
    def timer_callback(self):
        if self.index < len(self.images):
            cv_image_flat = self.images[self.index]
            img_msg = UInt8MultiArray()
            img_msg.data = cv_image_flat
            img_msg.layout = MultiArrayLayout()
            img_msg.layout.dim = []
            img_msg.layout.dim.append(MultiArrayDimension())
            img_msg.layout.dim[0].label = "height"
            img_msg.layout.dim[0].size = self.img_shape[0]
            img_msg.layout.dim[0].stride = self.img_shape[0] * self.img_shape[1] * self.img_shape[2]
            img_msg.layout.dim.append(MultiArrayDimension())
            img_msg.layout.dim[1].label = "width"
            img_msg.layout.dim[1].size = self.img_shape[1]
            img_msg.layout.dim[1].stride = self.img_shape[1] * self.img_shape[2]
            img_msg.layout.dim.append(MultiArrayDimension())
            img_msg.layout.dim[2].label = "channels"
            img_msg.layout.dim[2].size = self.img_shape[2]
            img_msg.layout.dim[2].stride = self.img_shape[2]
            self.publisher.publish(img_msg)
            self.index += 1
        else:
            self.index = 0
def main(args=None):
    rclpy.init(args=args)
    roller = Roller()
    rclpy.spin(roller)
    roller.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    