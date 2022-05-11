#! /usr/bin/env python2.7

import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import mavros
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget as PT
from std_msgs.msg import Float32
from tf import transformations as tr
import mavros_msgs.msg
from mavros import setpoint as SP
from simple_pid import PID

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        img = self.frame.copy() 
        
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(img, self.dictionary, parameters=self.parameters)
        # Get the transformation matrix to the marker
        if markerCorners:

            markerSize = 2.0
            axisLength = 3.0

            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners, markerSize, self.K, self.distCoeffs)
            
            # Draw the axis on the marker
            frame_out = cv.aruco.drawAxis( img, self.K, self.distCoeffs, rvecs, tvecs, axisLength)
            
            rotMat = tr.euler_matrix(np.pi / 2.0, 0, 0)
            rotMat = rotMat[0:3, 0:3]
            tvecs = np.matmul(rotMat, tvecs[0][0].T)
            rotMat, _ = cv.Rodrigues(rvecs)
            eul = tr.euler_from_matrix(rotMat)

            yaw_angle = self.pos[3] - eul[2] 

            # Publish the position of the marker
            marker_pos = PT()
            marker_pos.position.x = tvecs[0]
            marker_pos.position.y = - tvecs[2]
            marker_pos.position.z = tvecs[1]
            marker_pos.yaw = eul[2]  * np.pi / 180.0
            # self.aruco_marker_pos_pub.publish(marker_pos)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_out, "bgr8"))
        except CvBridgeError as e:
            print e

if __name__ == '__main__':
    try:
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Start cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()