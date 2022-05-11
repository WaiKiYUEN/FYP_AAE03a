#! /usr/bin/env python2.7

import rospy
import mavros
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget as PT
from geometry_msgs.msg import Vector3, TwistStamped
from drone_aruco_landing.srv import goto_aruco, goto_arucoResponse, land_aruco, land_arucoResponse, target_local_pos
from std_msgs.msg import Float32
from tf import transformations as tr
import mavros_msgs.msg
import mavros.command
from mavros import setpoint as SP
from simple_pid import PID
import tf

class ArucoNavigationController():

    def __init__(self):
        ''' Class that acts as a server for the goto_aruco service and the land_aruco service '''

        # init node
        rospy.init_node('aruco_navigation')
        mavros.set_namespace('mavros')

        # Setup Subscribers
        ## Marker pos
        aruco_pos = rospy.Subscriber('/aruco_marker_pos', PT, self._arucoCallback)
        ## mavros state
        state_sub = rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self._state_callback)
        # /mavros/local_position/pose
        local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self._local_position_callback)

        # Setup publishers
        # /mavros/setpoint_velocity/cmd_vel
        self.cmd_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        # setup services
        # /mavros/cmd/arming
        self.set_arming = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        # /mavros/set_mode
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)      
        # goto_pos_services
        self.goto_loc_pos_serv = rospy.ServiceProxy("target_local_pos", target_local_pos)

        # Initialize the service servers
        goto_aruco_serv = rospy.Service('goto_aruco', goto_aruco, self.GotoAruco)
        land_aruco_serv = rospy.Service('land_aruco', land_aruco, self.LandAruco)

        # Initialize variables
        self.local_pos = [0.0] * 4
        self.pos = [0.0] * 4
        self.markerPos = [0.0] * 4
        self.UAV_state = mavros_msgs.msg.State()
        self.markerHeight = 8 # Height above the marker

        # Setup rate
        self.rate = rospy.Rate(30)
        rospy.sleep(1)
        rospy.spin()

    def _arucoCallback(self, msg):
        ''' Callback for the aruco marker POS '''
        self.markerPos[0] = msg.position.x
        self.markerPos[1] = msg.position.y
        self.markerPos[2] = msg.position.z
        self.markerPos[3] = msg.yaw

        self.markerPos = np.array(self.markerPos)
    
    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided

    def _local_position_callback(self, topic):
        # Position data
        self.local_pos[0] = topic.pose.position.x
        self.local_pos[1] = topic.pose.position.y
        self.local_pos[2] = topic.pose.position.z
        # Orientation data
        (r, p, y) = tf.transformations.euler_from_quaternion([topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w])
        self.local_pos[3] = y
    
    def GotoAruco(self, req):
        ''' Goto the aruco marker '''
        rospy.loginfo('Going to aruco marker')
        timeOut = req.timeOut
        if timeOut == 5:
            self.markerHeight = 8
        elif timeOut == 12:
            self.markerHeight = 3
        elif timeOut == 7:
            self.markerHeight = 0.4
        new_sp = TwistStamped()
        while self.UAV_state.mode != "OFFBOARD" :
            rospy.sleep(0.1)
            self.set_mode(0, 'OFFBOARD')
            # Publish something to activate the offboard mode
            self.cmd_vel_pub.publish(new_sp)
        
        if not mavros.command.arming(True) :
            mavros.command.arming(True)
            
        ts = rospy.Time.now()

        
        xPID = PID(.5, 0.1, 0.01, output_limits=(-1.3, 1.3), setpoint=0.0)
        yPID = PID(.5, 0.1, 0.01, output_limits=(-1.3, 1.3), setpoint=0.0)
        zPID = PID(.2, 0.0, 0.05, output_limits=(-0.6, 0.6), setpoint=self.markerHeight)
        yawPID = PID(.2, 0.0, 0.0, output_limits=(-1.0, 1.0), setpoint=0.0)
        
        dist = -1
        failtime = 0
        #while np.linalg.norm(self.markerPos[0:3] - np.array([0.0, 0.0, -self.markerHeight])) > 0.5 and (rospy.Time.now() - ts < rospy.Duration(timeOut)):
        while (rospy.Time.now() - ts < rospy.Duration(timeOut)):        
            if timeOut == 7:
                new_sp = TwistStamped()
                new_sp.twist.linear.x = xPID(-self.markerPos[1])
                new_sp.twist.linear.y = yPID(self.markerPos[0] + 0.35)
                new_sp.twist.linear.z = zPID(-self.markerPos[2])
                new_sp.twist.angular.z = yawPID(self.markerPos[3])
            else:
                new_sp = TwistStamped()
                new_sp.twist.linear.x = xPID(-self.markerPos[1])
                new_sp.twist.linear.y = yPID(self.markerPos[0])
                new_sp.twist.linear.z = zPID(-self.markerPos[2])
                new_sp.twist.angular.z = yawPID(self.markerPos[3])
            
            #print(np.linalg.norm(self.markerPos[0:3] - np.array([0.0, 0.0, -self.markerHeight])))
            self.cmd_vel_pub.publish(new_sp)
            '''
            tar_pos = mavros_msgs.msg.PositionTarget(header=mavros.setpoint.Header(frame_id="att_pose", stamp=rospy.Time.now()), )
            tar_pos.position.x = self.local_pos[0] + self.markerPos[1]
            tar_pos.position.y = self.local_pos[1] - self.markerPos[0]
            tar_pos.position.z = self.markerHeight
            tar_pos.yaw = self.markerPos[3]
            try:
                dist = self.goto_loc_pos_serv(tar_pos)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                print("Service busy")
            '''
            failtest = self.markerPos[1]
            if failtest == self.markerPos[1]:
                failtime = failtime + 1
            if failtest > 5:
                return -1
        return goto_arucoResponse(np.linalg.norm(self.markerPos[0:3] - np.array([0.0, 0.0, 0.0])))
    
    def LandAruco(self, req):
        ''' Land on the aruco marker '''

        rospy.loginfo('Landing on the aruco marker')
        timeOut = req.timeOut

        new_sp = TwistStamped()
        while self.UAV_state.mode != "OFFBOARD" :
            rospy.sleep(0.1)
            self.set_mode(0, 'OFFBOARD')
            # Publish something to activate the offboard mode
            self.cmd_vel_pub.publish(new_sp)
        
        if not mavros.command.arming(True) :
            mavros.command.arming(True)
            
        ts = rospy.Time.now()

        xPID = PID(.6, 0.1, 0.01, output_limits=(-1.8, 1.8), setpoint=0.0)
        yPID = PID(.6, 0.1, 0.01, output_limits=(-1.8, 1.8), setpoint=0.0)
        zPID = PID(.5, 0.0, 0.05, output_limits=(-1, 1), setpoint=0.0)
        yawPID = PID(.2, 0.0, 0.0, output_limits=(-1.0, 1.0), setpoint=0.0)
        
        prev_height = self.markerPos[2]
        ts2 = rospy.Time.now()

        while (rospy.Time.now() - ts < rospy.Duration(timeOut)):
                
            new_sp = TwistStamped()
            new_sp.twist.linear.x = xPID(-self.markerPos[1])
            new_sp.twist.linear.y = yPID(self.markerPos[0])
            new_sp.twist.linear.z = zPID(-self.markerPos[2])
            new_sp.twist.angular.z = yawPID(self.markerPos[3])
            
            if self.markerPos[2] != prev_height:
                ts2 = rospy.Time.now()
                prev_height = self.markerPos[2]
            
            if rospy.Time.now() - ts2 > rospy.Duration(1.0):
                break
            #print(np.linalg.norm(self.markerPos[0:3] - np.array([0.0, 0.0, -self.markerHeight])))

            self.cmd_vel_pub.publish(new_sp)
        return land_arucoResponse(np.linalg.norm(self.markerPos[0:3] - np.array([0.0, 0.0, 0.0])))

###################################################################################################
if __name__ == "__main__":
    ANC = ArucoNavigationController()
    
    
