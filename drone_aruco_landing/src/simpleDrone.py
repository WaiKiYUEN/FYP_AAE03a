#!/usr/bin/env python2.7

# import ROS libraries
from operator import truediv
from os import times
from sqlite3 import Time
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
from sensor_msgs.msg import NavSatFix, Imu
from mavros_msgs.srv import CommandLong
from drone_aruco_landing.srv import target_global_pos, target_global_posResponse, target_local_pos, target_local_posResponse, goto_aruco, goto_arucoResponse, land_aruco, land_arucoResponse 
import sys
import signal
from geometry_msgs.msg import Vector3, TwistStamped
from std_msgs.msg import Float32
import math
import numpy as np
import utm
import tf

class SimpleDrone():
    """
        Class that connects with a drone using mavros at the gazebo simulation
    """

    def __init__(self):
        
        # Initialize variables
        self.local_pos = [0.0] * 4
        self.global_pos = [0.0] * 4

        rospy.init_node("uav_node")

        self.rate = rospy.Rate(30)
        mavros.set_namespace('mavros')

        signal.signal(signal.SIGINT, self.signal_handler)

        self.current_pose = Vector3()
        self.UAV_state = mavros_msgs.msg.State()

        self.stage = 0

        # setup subscribers
        # /mavros/state
        state_sub = rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, self._state_callback)

        # /mavros/local_position/pose
        local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self._local_position_callback)

        # /mavros/setpoint_raw/target_local
        setpoint_local_sub = rospy.Subscriber(mavros.get_topic('setpoint_raw', 'target_local'), mavros_msgs.msg.PositionTarget, self._setpoint_position_callback)
        # /mavros/global_position/global
        global_position_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, self._global_position_callback)
        # /mavros/imu/data
        rospy.Subscriber('/mavros/imu/data', Imu, self.IMU_callback) 

        # setup publisher
        # /mavros/setpoint/position/local
        self.setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)
        # /mavros/setpoint_velocity/cmd_vel
        self.cmd_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        # setup services
        # /mavros/cmd/arming
        self.set_arming = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        # mavros/cmd/command
        self.command_srv = rospy.ServiceProxy('mavros/cmd/command', mavros_msgs.srv.CommandLong)
        # /mavros/set_mode
        self.set_mode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)      
        # goto_pos_services
        self.goto_glo_pos_serv = rospy.ServiceProxy("target_global_pos", target_global_pos)
        self.goto_loc_pos_serv = rospy.ServiceProxy("target_local_pos", target_local_pos)
        # aruco based services
        self.goto_aruco_serv = rospy.ServiceProxy('goto_aruco', goto_aruco)
        self.land_aruco_serv = rospy.ServiceProxy('land_aruco', land_aruco)

        self.setpoint_msg = mavros.setpoint.PoseStamped(
            header=mavros.setpoint.Header(frame_id="att_pose", stamp=rospy.Time.now()), )
        
        # wait for FCU connection
        while (not self.UAV_state.connected):
            self.rate.sleep()

        rospy.loginfo("Uav was successfully connected")

        self.InputHandler()

        rospy.spin() #WAS NOT USED

    def signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)

    def IMU_callback(self, data):
        ''' IMU data subscriber that triggers the parachute in case of failure'''
        # Read the linear_accelaration on z-axis
        acc_z = data.linear_acceleration.z

        # if the acceleration is too low triger the parachute
        if acc_z < 1.0: #CHANGED FROM 6.0
            self.command_srv(broadcast=False, command=185, param1=1.0)

    def _state_callback(self, topic):
        self.UAV_state.armed = topic.armed
        self.UAV_state.connected = topic.connected
        self.UAV_state.mode = topic.mode
        self.UAV_state.guided = topic.guided

    def _setpoint_position_callback(self, topic):
        pass

    def _local_position_callback(self, topic):
        # Position data
        self.local_pos[0] = topic.pose.position.x
        self.local_pos[1] = topic.pose.position.y
        self.local_pos[2] = topic.pose.position.z

         # Orientation data
        (r, p, y) = tf.transformations.euler_from_quaternion([topic.pose.orientation.x, topic.pose.orientation.y, topic.pose.orientation.z, topic.pose.orientation.w])
        self.local_pos[3] = y
        self.global_pos[3] = y
        
    def _global_position_callback(self, data):
        self.global_pos[0] = data.latitude
        self.global_pos[1] = data.longitude
        self.global_pos[2] = data.altitude

    def _set_pose(self, pose, x, y, z):
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.header = mavros.setpoint.Header( frame_id="att_pose", stamp=rospy.Time.now())

    def update_setpoint(self):
        pass
   
    def Hover(self):
        """
            Used to keep the altitude and position of the drone stable once activated
        """
        print("Hover", self.set_mode(0, 'AUTO.LOITER'))
    
    def GotoPos(self, pos, pos_type = "local"):
        """
            Gets the drone to the defined pos,
            @pos_type = "local" for movement on the local coordinate system, pos[4] = [x, y, z, Yaw]
            @pos_type = "global" for movement using the global coordinate system pos[4] = [Lat, Lon, Alt, Yaw]
        """

        # Deactivate the hover function if activated
        dist = -1  
        if (pos_type == "local"):
            loc_pos = mavros_msgs.msg.PositionTarget(header=mavros.setpoint.Header(frame_id="att_pose", stamp=rospy.Time.now()), )
            # Position
            loc_pos.position.x = pos[0]
            loc_pos.position.y = pos[1]
            loc_pos.position.z = pos[2]

            # Orientation
            loc_pos.yaw = pos[3]

            # Call the service
            try:
                dist = self.goto_loc_pos_serv(loc_pos)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                print("Service busy")
                
        if (pos_type == "global"):
            glob_pos = mavros_msgs.msg.GlobalPositionTarget(header=mavros.setpoint.Header(frame_id="att_pose", stamp=rospy.Time.now()), )
            
            # Position 
            glob_pos.latitude = pos[0]
            glob_pos.longitude = pos[1]
            glob_pos.altitude = pos[2]
            
            # Orientation
            glob_pos.yaw = pos[3]
            try:
                dist = self.goto_glo_pos_serv(glob_pos)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                print("Service busy")
        
        return dist
    
    def ArucoTrace(self, timeOut):
        dist = -1
        if timeOut == 10:
            sub_timer = 5
        elif timeOut == 12:
            sub_timer = 12
        elif timeOut == 14:
            sub_timer = 7
        timestart = rospy.Time.now()
        timeOut = float(timeOut)
        while (rospy.Time.now() - timestart < rospy.Duration(timeOut)):
            dist = self.goto_aruco_serv(sub_timer)
            if sub_timer == 7 and dist.dist < 0.55:
                return dist
        if dist == -1:
            return -1
        else:
            return dist

    def Land(self, pos = []):
        ''' Land the UAV at its current\given position '''
        if len(pos) > 0.01:
            self.GotoPos(pos)
        
        self.set_mode(0, 'AUTO.LAND')
        #mavros.command.arming(False)

    def Return(self):
        ''' Return to the home pos and land'''
        self.set_mode(0, 'AUTO.RTL')
        #mavros.command.arming(False)

    #hands-free time! ;)
    def auto(self):
        #stage of UAV: 0-hover; 1-follow; 2-descend; 3-shutdown
        flag = False
        found = -1
        timeOut = 42
        while not flag:
            if self.stage == 0:
                print("Stage 0 engaged, traveling to target proximity.")
                int_pos = self.local_pos
                int_pos[0] = float(0)
                int_pos[1] = float(0)
                int_pos[2] = float(12)
                int_pos[3] = float(0)
                dist = self.GotoPos(int_pos)
                self.Hover()
                print("Stage 0 done.")
                self.stage = 1
            if self.stage == 1:
                print("Stage 1 engaged, following the target.")
                timeOut = 10
                found = self.ArucoTrace(timeOut)
                if found != -1:
                    print("Stage 1 done.")
                    self.stage = 2
                else:
                    self.Hover()
                    print("Target not found, returning to origin.")
                    self.stage = 0
            if self.stage == 2:
                print("Stage 2 engaged, descending to the target.")
                timeOut = 12
                found = self.ArucoTrace(timeOut)
                if found != -1:
                    print("Stage 2 done.")
                    self.stage = 3
                    print("Stage 3 engaged, landing on the target.")
                else:
                    self.Hover()
                    print("Target not found, returning to origin.")
                    self.stage = 0
            if self.stage == 3:
                timeOut = 14
                found = self.ArucoTrace(timeOut)
                print("Dist: " + str(found.dist))
                if found.dist < 0.59:
                    print("!!!I am now landing!!!")
                    self.land_aruco_serv(30)
                    self.Land()
                    print("Process completed!")
                    flag = True
                    self.stage = 0
                elif found == -1:
                    self.Hover()
                    print("Target not found, returning to origin.")
                    self.stage = 0


    
    def Help(self):
        """ Print the instructions"""
        print("----- available inputs -----")
        print("exit -> to close the swarm commander")
        print("auto -> complete system automation")
        print("goto x y z yaw  -> uav will go to the specified location location")
        print("goto aruco [timeout] -> the uav will got to the aruco marker, timeOut is optional")
        print("land [x y z yaw] -> uav will land at its current position or at the optional x, y, z, yaw")
        print("land aruco -> uav will land on the aruco marker")
        print("return -> uav will return to its home position and land")
        print("----------------------------")
    
    def InputHandler(self):
        """ Used to handle the keyboard inputs from the terminal"""
        done = False
        WirePos = mavros_msgs.msg.PositionTarget()
        swarmEffect = False
        while not done:
            inp = raw_input("Type your command > ")
            inp = inp.split()
            print(len(inp), (inp))
            
            if len(inp) == 1:
                if inp[0] == 'exit':
                    done = True
                elif inp[0] == 'return':
                    self.Return()
                elif inp[0] == 'land' and len(inp) == 1:
                    self.Land()
                elif inp[0] == 'auto':
                    self.stage = 0
                    self.auto()
                else:
                    self.Help()
            elif len(inp) > 1:
                pos = []
                if (len(inp) == 5):
                    pos.append(float(inp[1]))
                    pos.append(float(inp[2]))
                    pos.append(float(inp[3]))
                    pos.append(float(inp[4]))

                if inp[0] == 'goto':
                    if len(inp) == 5:
                        dist = self.GotoPos(pos)
                        self.Hover()
                    elif inp[1] == 'aruco':
                        timeOut = 15
                        if len(inp) == 3:
                            timeOut = inp[2]
                        dist = self.goto_aruco_serv(timeOut)
                        self.Hover()
                    else:             
                        self.Help()
                        continue   
                    print("Position reached, distance ", dist) 
                elif inp[0] == 'land':
                    if len(inp) == 5:
                        self.Land(pos)
                    elif inp[1] == 'aruco':
                        # First go to the aruco marker
                        dist = self.goto_aruco_serv(60)
                        rospy.loginfo('Distance to marker ' + str(dist.dist))
                        if dist.dist < 1.0:
                            self.land_aruco_serv(60)
                            self.Land()
                        else:
                            rospy.logerr('Marker too far')
                            self.Hover()
                            continue
                    else:
                        self.Help()
                else:
                    self.Help()
            else:
                self.Help()

###################################################################################################        
if __name__ == "__main__":
    SD = SimpleDrone()
    #SD2 = SimpleDrone(2)
    #SD2.GotoLocalPos(5, 5, 10)
    #SD2.SimpleMovement()
    #SD1.GotoGlobPos(55.43600, 10.46091, 540)
