#!/usr/bin/env python

import rospy
import mavros
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget as PT
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations as tr
import mavros_msgs.msg
from mavros import setpoint as SP
from simple_pid import PID
import cv2.aruco as aruco

class image_convert_pub:

  def __init__(self):
    mavros.set_namespace('mavros')

    #Init pub
    self.image_pub = rospy.Publisher("/aruco_marker_img",Image, queue_size=1)
    self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
    self.aruco_marker_pos_pub = rospy.Publisher('/aruco_marker_pos', PT, queue_size=1)
    self.bridge = CvBridge()
    
    #Init sub
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
    local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self._local_position_callback)

    #Init var
    self.frame = np.zeros((240, 320, 3), np.uint8)
    self.pos = [0.0] * 4
    self.markerPos = [0.0] * 4


    #cam intristic param
    self.K = np.array([277.191356, 0.0, 160.5, 0.0, 277.191356, 120.5, 0.0, 0.0, 1.0]).reshape(3,3)
    self.distCoeffs = np.array([0.0] * 5)

    #rate
    self.rate = rospy.Rate(30)
    self.rate.sleep()

  def _local_position_callback(self, topic):
    self.pos[0] = topic.pose.position.x
    self.pos[1] = topic.pose.position.y
    self.pos[2] = topic.pose.position.z

    (r, p, y) = tr.euler_from_quaternion([topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w])
    self.pos[3] = y

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    markers_img, ids_list = self.detect_aruco(cv_image)

    #id pub
    if ids_list is None:
      self.id_pub.publish(ids_list)
    else:
      ids_str = ''.join(str(e) for e in ids_list)
      self.id_pub.publish(ids_str)

    #img pub
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_aruco(self,img):
    #def
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    markerSize = 2.0
    axisLength = 3.0

    #detect
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    
    #process
    """
      #detect the aruco markers and display its aruco id.
      output = aruco.drawDetectedMarkers(img, corners, ids)
    """
    if markerSize:
      rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSize, self.K, self.distCoeffs)

      #draw axis
      output = cv2.aruco.drawAxis(img, self.K, self.distCoeffs, rvecs, tvecs, axisLength)
      rotMat = tr.euler_matrix(np.pi / 2.0, 0, 0)
      rotMat = rotMat[0:3, 0:3]
      tvecs = np.matmul(rotMat, tvecs[0][0].T)
      rotMat, _ = cv2.Rodrigues(rvecs)
      eul = tr.euler_from_matrix(rotMat)
      yaw_angle = self.pos[3] - eul[2]

      #aruco pos pub
      marker_pos = PT()
      marker_pos.position.x = tvecs[0]
      marker_pos.position.y = - tvecs[2]
      marker_pos.position.z = tvecs[1]
      marker_pos.yaw = eul[2]  * np.pi / 180.0
      self.aruco_marker_pos_pub.publish(marker_pos)

    return output, ids

def main():
  print("Initializing ROS-node")
  rospy.init_node('marker_detector', anonymous=True)
  print("Bring the aruco marker in front of camera")
  ic = image_convert_pub()
  rospy.spin()

if __name__ == '__main__':
    main()