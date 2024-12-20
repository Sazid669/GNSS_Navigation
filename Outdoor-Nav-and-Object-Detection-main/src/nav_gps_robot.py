#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, Point
import math
import yaml
import rospkg
import time as tm   
import actionlib    
from math import radians as deg2rad
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import tf
from utils.common_func import CommonFunc
from utils.Static_tranform import PublishStaticTransform 
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import MagneticField
import cv2

class GPSController:
    def __init__(self):
        rospy.init_node('gps_controller')
    
        self.radius_earth = rospy.get_param('~radius', 6371000)    
        self.bridge = CvBridge()
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.1) 
        self.goal_tolerance = rospy.get_param('~goal_tolerance',12)   
        self.agle_tolerance = rospy.get_param('~angle_tolerance', 20) 
        self.goals_file = rospy.get_param('~goals_file', 'config/goals.yaml')
        self.goals = self.Get_goals()  
        self.current_goal_index = 0
        self.traj_points = []
        self.tf_listener = tf.TransformListener()
        self.prev_time = rospy.Time.now()   
        self.prev_error_angle = 0
        self.integral_angle = 0
        self.prev_error_distance = 0
        self.integral_distance = 0 
        self.obstacle_detected = False  
        self.traj_points = []
        self.intial_orient = None
        self.current_heading = 0.0
        self.previous_x = None
        self.previous_y = None
        self.instant_pose = None
        self.initt = None
        self.ref_lat =None
        self.ref_lon = None
        self.reference_set = False
        self.ref_heading = 0.0
        self.pose_now_x=0.0
        self.pose_now_y=0.0
        self.current_theta=0.0
        self.declination_offsets = {'x': -0.1908, 'y': -0.0518, 'z': -0.5559}  #
        self.linear_vel_limit = 0.7
        self.angular_vel_limit = 1.0
        self.CommonFunc = CommonFunc()   
        # self.StaticMapOdom = Static_tranform.PublishStaticTransform()  
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_marker = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)
        self.gps_pathM =  rospy.Publisher('/gps_traj', Marker, queue_size=10)
        self.goal_pub  = rospy.Publisher('/given_goal', PoseStamped, queue_size=10)
        self.current_pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)
        self.GPS_ODOM_PUB = rospy.Publisher('/gps_odom', Odometry, queue_size=10)  # GPS +IMU
        self.gps_subscriber = rospy.Subscriber('/gnss', NavSatFix, self.gps_callback)  # Real robot
        # rospy.Subscriber('/camera/depth/image_raw', Image, self.camera_clk)
        self.camera_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.camera_clk) # Real robot
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # self.orient_sub = rospy.Subscriber('/imu/data', Imu, self.current_orientation) #Does not work
        self.orient_sub = rospy.Subscriber('/imu/mag', Vector3Stamped, self.imu_mag) 
        
        rospy.Subscriber('/gps_odom', Odometry, self.gps_odom)
          
    def Get_goals(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('scout_mini_project')
        goals_file = rospy.get_param('~goals_file', package_path + '/config/goals.yaml')
        with open(goals_file, 'r') as file:
            goals_data = yaml.safe_load(file)
            return goals_data['goals']
         
    def gps_callback(self, msg: NavSatFix):    
        
        if not self.reference_set:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.reference_set = True
            rospy.loginfo(f"Ref GPS: {self.ref_lat}, Long: {self.ref_lon}")
            return  
        
        if self.current_goal_index>=len(self.goals):
            rospy.loginfo("All goals reached!! ")
            self.stop_robot()
            return
        
        if self.obstacle_detected:
            tm.sleep(0.05)
            self.avoid_obstacle()
            return  
        
        ref_E, ref_N = self.CommonFunc.GPS2UTM(self.ref_lat, self.ref_lon)
        current_UTM_E, current_UTM_N = self.CommonFunc.GPS2UTM(msg.latitude, msg.longitude)
        
        orient_diff = self.CommonFunc.wrap_angle(self.current_heading - self.ref_heading)
        # orient_diff = self.CommonFunc.wrap_angle(self.CommonFunc.angle(ref_E, ref_N, current_UTM_E, current_UTM_N))
        current_x, current_y = self.CommonFunc.UTM2ODOM(msg.latitude, 
                                       msg.longitude, 
                                       self.ref_lat, 
                                       self.ref_lon, 
                                       orient_diff)
      
        self.publish_current_pose(current_x, current_y)
        self.GPSTraj_marker(current_x, current_y)
        self.GPS_ODOM(current_x, current_y)
        
        Temp_goal_lat, Temp_goal_lon = self.goals[self.current_goal_index]
        goal_E, goal_N = self.CommonFunc.GPS2UTM(Temp_goal_lat, Temp_goal_lon)
        GoalOrient_ref = self.CommonFunc.angle(ref_E, ref_N, goal_E, goal_N)    
        print(f"GOal_ref_before_wrapping:, {GoalOrient_ref}")
        
        GoalOrient_ref = self.CommonFunc.wrap_angle(self.CommonFunc.angle(ref_E, ref_N, goal_E, goal_N)) 
        print(f"After wrapping angle: {GoalOrient_ref}")
        
        self.goal_x, self.goal_y = self.CommonFunc.UTM2ODOM(Temp_goal_lat, 
                                                            Temp_goal_lon, 
                                                            self.ref_lat, 
                                                            self.ref_lon, 
                                                            GoalOrient_ref)
        
        self.goal_pose(self.goal_x, self.goal_y)  
        distance_to_goal = self.CommonFunc.distance(current_x, current_y, self.goal_x, self.goal_y)    
        angle_to_goal = self.CommonFunc.angle(current_x, current_y, self.goal_x, self.goal_y) 
        # print("goal_orient", angle_to_goal)  
                        
        if distance_to_goal <= self.goal_tolerance:
            rospy.loginfo(f"GOAL!! of {self.current_goal_index} reached.")
            self.stop_robot()
            self.current_goal_index += 1
            return
    
        if self.current_heading is None:
            rospy.loginfo("Waiting for IMU......")
            return  
       
        error_angle = self.CommonFunc.wrap_angle(angle_to_goal)
        
        if abs(error_angle) > self.agle_tolerance:
            twist = Twist()
            twist.linear.x = 0.0
            # twist.angular.z = self.pid_control(error_angle, self.prev_error_angle, self.integral_angle, 'angle')4
            twist.angular.z = self.angular_vel_limit * error_angle 
            self.cmd_vel_pub.publish(twist)
            return
        
        # linear_vel = self.pid_control(distance_to_goal, self.prev_error_distance, self.integral_distance, 'distance')    
        twist = Twist()
        twist.linear.x =  min(self.linear_vel_limit, distance_to_goal) 
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.prev_error_angle = error_angle
        self.integral_angle += error_angle
        self.prev_error_distance = distance_to_goal
        self.integral_distance += distance_to_goal
        self.previous_x = current_x
        self.previous_y = current_y
    
            
    def camera_clk(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        cv_img_resized = cv2.resize(cv_img, (cv_img.shape[1] // 2, cv_img.shape[0] // 2))
        depth_in_meters = cv_img_resized / 1000.0
        # median filtering--> reduce noise
        cv_img_filtered = cv2.medianBlur(depth_in_meters.astype(np.float32), 5)

        valid_depth = cv_img_filtered[(cv_img_filtered > 0.05) & (cv_img_filtered < 10.0)]
        if len(valid_depth) == 0:
            return

        mean_depth = np.mean(valid_depth)
        std_depth = np.std(valid_depth)
        valid_depth = valid_depth[np.abs(valid_depth - mean_depth) < 2 * std_depth]

        # Finally! 
        min_distance = np.min(valid_depth)
        self.obstacle_detected = min_distance <= self.obstacle_threshold
        if self.obstacle_detected:
            rospy.logwarn("Obstacle detected!")
        else:
            rospy.loginfo("No obstacle detected.")

    def imu_mag(self, msg: Vector3Stamped):
        mag_x_raw = msg.vector.x - self.declination_offsets['x']
        mag_y_raw = msg.vector.y - self.declination_offsets['y']
        heading = math.atan2(-mag_y_raw, mag_x_raw)  # ENU convention
        heading = math.degrees(heading)
        heading = (heading + 360) % 360  

        alpha = 0.1 
        if self.current_heading is None:
            self.current_heading = heading
        else:
            self.current_heading = alpha * heading + (1 - alpha) * self.current_heading

    def odom_callback(self, msg: Odometry):
        self.pose_now_x = msg.pose.pose.position.x
        self.pose_now_y = msg.pose.pose.position.y   
        self.instant_pose = (self.pose_now_x, self.pose_now_y)    
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_theta = tf.transformations.euler_from_quaternion(orientation_list)
        self.publish_current_pose(self.pose_now_x, self.pose_now_y)
        self.ODOMTraj_marker(self.pose_now_x, self.pose_now_y) 
            
    def gps_odom(self, messsage:Odometry):
        gps_pose_x = messsage.pose.pose.position.x
        gps_pose_y = messsage.pose.pose.position.y
        
        
    def TransformGps2Baselink(self, gpsX, gpsY):
        try:
            self.tf_listener.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))
            gps_pose = PoseStamped()
            gps_pose.header.frame_id = "odom"
            gps_pose.pose.position.x = gpsX
            gps_pose.pose.position.y = gpsY
            gps_pose.pose.position.z = 0.0
            gps_pose.pose.orientation.x = 0.0
            gps_pose.pose.orientation.y = 0.0
            gps_pose.pose.orientation.z = 0.0
            gps_pose.pose.orientation.w = 1.0
            base_link_pose = self.tf_listener.transformPose("base_link", gps_pose)
            return base_link_pose.pose.position.x, base_link_pose.pose.position.y
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Transform lookup failed")
            return None, None
        

    def avoid_obstacle(self):
        rate = rospy.Rate(10)
        while self.obstacle_detected and not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.4
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("Cleared.")
        

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def pid_control(self, error, prev_error, integral, type):
        Kp = rospy.get_param(f'~Kp_{type}', 0.02)
        Ki = rospy.get_param(f'~Ki_{type}', 0.001)
        Kd = rospy.get_param(f'~Kd_{type}', 0.005)
        current_time = rospy.Time.now() 
        delta_time = (current_time - self.prev_time).to_sec()       
        derivative = error - prev_error
        output = Kp * error + Ki * integral * delta_time + Kd * derivative / delta_time 
        self.prev_time = current_time   
        return output

    def publish_current_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.pose_now_x
        pose.pose.position.y = self.pose_now_y  
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.current_pose_pub.publish(pose)
 
    def goal_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        angle_to_goal = self.CommonFunc.angle(self.pose_now_x, self.pose_now_y, x, y)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle_to_goal)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.goal_pub.publish(pose)
          
        
    # TRAJECTORY
    def ODOMTraj_marker(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        self.traj_points.append(point)
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "Path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  
        marker.color.a = 1.0  
        marker.color.r = 0.9
        marker.color.g = 0.5
        marker.color.b = 0.0 
        marker.points = self.traj_points
        
        self.path_marker.publish(marker)
        
        
    def GPSTraj_marker(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        self.traj_points.append(point)
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "odom"
        marker.ns = "Path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  
        marker.color.a = 0.0  
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0 
        marker.points = self.traj_points
        self.gps_pathM.publish(marker)
        
        
        
        
    def GPS_ODOM(self,X:float, Y:float):
        #ODOM_PUB
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom" 
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = X
        odom_msg.pose.pose.position.y = Y
        odom_msg.pose.pose.position.z = 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_heading)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]
        
        #
        odom_msg.pose.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,  # 
                                    0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01] # 
        self.GPS_ODOM_PUB.publish(odom_msg)
            
    def spin(self):
        rospy.spin()   

if __name__ == '__main__':
    if rospy.is_shutdown():
        rospy.logerr("ROS-MASTER IS NOT STARTED" )
    call_node = GPSController()
    call_node.spin()