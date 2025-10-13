import numpy as np 
import rclpy 
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
import os

class Republisher(Node):
    def __init__(self):
        super().__init__('republisher')
        #TODO: Create a subscriber to 'image_array' topic and a publisher to 'image' topic
        self.image_array_sub = self.create_subscription(UInt8MultiArray, 'image_array', self.listener_callback,10)
        self.publisher = self.create_publisher(Image, '/wagbaba', 10 )
    def listener_callback(self, msg):
        """Convert UInt8MultiArray message to Image message and publish it."""
        """For more information on message formats, visit:https://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt8MultiArray.html."""
        """https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html."""

        #TODO: find out the height, width and channels from msg.layout.dim

        height = msg.layout.dim[0].size
        width = msg.layout.dim[1].size
        channels = msg.layout.dim[2].size
        img_data = np.array(msg.data, dtype=np.uint8).reshape((height, width, channels))
        img_msg = Image()
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = 'rgb8'
        img_msg.data = img_data.tobytes()
        img_msg.step = width * channels

        #TODO: publish the Image message
        self.publisher.publish(img_msg)
        

def main(args=None):
    rclpy.init(args=args)
    republisher = Republisher()
    rclpy.spin(republisher)
    republisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    