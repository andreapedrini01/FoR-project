"""
file Vision.py
@author Federico Buzzini
@brief This file defines the class Vision and its methods to recognize lego blocks from ZED camera and communicate with different ROS node
@date 2024-01-23
"""

# -------------------------- IMPORT -------------------------
import os
import sys
from pathlib import Path
import cv2 as cv
import numpy as np
import rospy as ros
from motion.msg import pos
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from LegoDetect import LegoDetect

# ------------------------- GLOBAL CONSTANTS ------------------------

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
IMG_ZED = os.path.abspath(os.path.join(ROOT, "log/img_ZED_cam.png"))

OFFSET = 0.86 + 0.1
w_R_c = np.matrix([[0, -0.499, 0.866], [-1, 0, 0], [0, -0.866, -0.499]])
base_offset = np.array([0.5+0.0154, 0.35, 1.75])
x_c = np.array([-0.9, 0.24, -0.35])

# ------------------------- CLASS -------------------------

class Vision:

    def __init__(self):

        # Initialize ROS node
        ros.init_node('vision', anonymous=True)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        self.block_list = []


        # Flags
        self.pointcloud_ready = False
        self.receive_image_ready = True
        self.vision_ready = 0

        # Subscribers and publishers for ROS communication with other nodes
        self.pointcloud_sub = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)
        self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, self.receive_image)
        self.ack_sub = ros.Subscriber('/vision/ack', Int32, self.ackCallbak)

        self.ack_pub = ros.Publisher('/taskManager/stop', Int32, queue_size=1)
        self.pos_pub = ros.Publisher("/vision/pos", pos, queue_size=1)



    def receive_image(self, data):

        #Callback function whenever take image msg from ZED camera

        # Flag
        if not self.receive_image_ready:
            return
        self.receive_image_ready = False

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Save image to file for debugging purposes (optional)
        cv.imwrite(IMG_ZED, cv_image)
        legoDetect = LegoDetect(IMG_ZED)
        self.block_list = legoDetect.block_list

        self.pointcloud_ready = True

    def receive_pointcloud(self, msg):
        #Callback function whenever take point cloud msg from ZED camera

        # Flag
        if not self.pointcloud_ready:
            return
        self.pointcloud_ready = False

        self.pos_msg_list = []

        for block in self.block_list:

            # Get point cloud from ZED camera and transform it to world coordinates using the camera pose and the base offset of the robot arm
            for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=True, uvs=[block.center_point]):
                block.point_cloud = (data[0], data[1], data[2])
                block.point_world = w_R_c.dot(block.point_cloud) + x_c + base_offset

            # Show details
            block.show()

            # Create msg for pos_pub
            pos_msg = pos()
            pos_msg.class_id = block.class_id
            pos_msg.pitch = 0
            pos_msg.roll = 0
            pos_msg.yaw = 1.57
            pos_msg.x = block.point_world[0, 0]
            pos_msg.y = block.point_world[0, 1]
            pos_msg.z = block.point_world[0, 2]


            if pos_msg.z < OFFSET:
                self.pos_msg_list.append(pos_msg)

        print('\nVISON FINISHED!\nWAITING FOR MOTION TO ACK\n')
        self.vision_ready = 1
        self.send_pos_msg()

    def ackCallbak(self, ack_ready):
        #Callback function whenever take ack msg from taskManager node to check if it is ready to receive the position of the block

        if  ack_ready.data == 1 and self.vision_ready == 1:
            self.send_pos_msg()

    def send_pos_msg(self):
        #Send position message to the taskManager node

        try:
            pos_msg = self.pos_msg_list.pop()
            self.pos_pub.publish(pos_msg)
            print('\nPosition published:\n', pos_msg)
        except IndexError:
            print('\nALL DONE, NO MORE BLOCKS\n')



# ----------------------------- MAIN ----------------------------
# to run this file, open a terminal and type in the correct folder path the following command:
# python3 Vision.py


# Check if the current module is being run as the main program
if __name__ == '__main__':
    # Create an instance of the Vision class
    vision = Vision()

    try:
        # Start the ROS node and enter the event loop
        ros.spin()
    except KeyboardInterrupt:
        # Handle keyboard interrupt and print a message
        print("Shutting down")
