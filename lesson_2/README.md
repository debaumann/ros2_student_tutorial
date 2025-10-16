## Lesson 2

Pre-requisites: 

install ros2 jazzy following this guide https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Tutorial Contents:
- basic comminucation features of ROS2, namely publishers, subscribers and topics.
- ROS std_msgs and sensor_msgs
  - http://docs.ros.org/en/noetic/api/std_msgs/html/index-msg.html
  - http://docs.ros.org/en/noetic/api/sensor_msgs/html/index-msg.html
- ros2 topic cmd line
- ros2 interface cmd line
- launching and running nodes
- COLCON build tool

Tutorial Overview:

A provided node that will publish an encoded image under the topic name ```/encoded_image```
You must identify the encoded image mesage type and subscribe to the topic
in the callback you must figure how to extract the shape of the image height x width x channels and the data.

next create a publisher that publishes messages of the type ```sensor_msgs/msg/Image```
in the subscriber callback, use the metadata and the data to form an Image msg and publish it to a topic.

Next open RQT and visualize your Image topic and enjoy.

Tutorial Workflow:

1. in your workspace build the the talker_listener package using colcon build --symlink-install
2. Launch the img_pub node with ```ros2 launch talker_listener img_pub.launch.py
3. Use ```ros2 topic info,echo "topic"``` and ```ros2 interface show "msg type"```to gather information about the encoded array topic
4. in the republisher.py file, search for the TODO blocks and implement your solution
5. open RQT and in the top bar under Plugins, select Visualisations and select Image viewer. Enjoy


