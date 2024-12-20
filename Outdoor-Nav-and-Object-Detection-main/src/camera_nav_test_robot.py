#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import math
from utils.common_func import CommonFunc

class ObstacleAvoidance:
    def __init__(self):
        self.bridge = CvBridge()
        self.distance_threshold = 0.5
        
        self.current_x, self.current_y, self.current_theta = 0, 0, 0
        self.target_x, self.target_y = 0, 0 
        self.goal_tolerance = 0.2
        self.linear_vel_limit = 0.5
        self.angular_vel_limit = 1.0
        
        self.MOVE_PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.TRAJ_PUB = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)
        self.GOAL_POSEE = rospy.Publisher('/given_goal', PoseStamped, queue_size=10)
        self.CURRENT_POSE = rospy.Publisher('/current_pose', PoseStamped, queue_size=10)    
        # rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.DEPTH_CLBK) # Real robot
        rospy.Subscriber('/camera/depth/image_raw', Image, self.DEPTH_CLBK)
        rospy.Subscriber('/odom', Odometry, self.ODOM_READ)

        self.safe_to_move = True
        self.move_base_cmd = Twist()  
        self.trajectory_points = []
        self.CommonFunc = CommonFunc()    

    def DEPTH_CLBK(self, data: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        visible_reg = cv_img / 1000.0  # Convert depth to meters
        visible_img = visible_reg[visible_reg != 0]
        min_distance = np.min(visible_img) if visible_img.size > 0 else float('inf')

        if min_distance < self.distance_threshold:
            rospy.logwarn("Obstacle! Stopping Sh**.")
            self.safe_to_move = False
        else:
            self.safe_to_move = True
        self.SAFEMOVE()


    def SAFEMOVE(self):
        if self.safe_to_move:
            self.MOVE2GOAL()
        else:
            rotate_cmd = Twist()
            rotate_cmd.linear.x = 0.0
            rotate_cmd.angular.z = 0.5  
            self.MOVE_PUB.publish(rotate_cmd)


    def MOVE2GOAL(self): 
        delta_x = self.target_x - self.current_x
        delta_y = self.target_y - self.current_y
        distance_to_goal = self.CommonFunc.distance(self.current_x, self.current_y, self.target_x, self.target_y)        

        if distance_to_goal < self.goal_tolerance:
            self.MOVE_PUB.publish(Twist())  
            rospy.loginfo("Goal reached!")
            return

        goal_angle = math.atan2(delta_y, delta_x)
        angle_diff = goal_angle - self.current_theta

        angle_diff = self.CommonFunc.wrap_angle(angle_diff)

        move_cmd = Twist()
        move_cmd.linear.x = min(self.linear_vel_limit, distance_to_goal)  # !>= 0.5 m/s
        move_cmd.angular.z = self.angular_vel_limit * angle_diff  
        self.MOVE_PUB.publish(move_cmd)

    def ODOM_READ(self, data):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.current_theta = tf.transformations.euler_from_quaternion(orientation_list)
        self.letsee_current_pose()
        self.draw_trajectory(self.current_x, self.current_y)
        
        
    def put_goal_pose(self, goal_msg: PoseStamped):
        goal_msg.header.stamp = rospy.Time.now()  # 
        self.GOAL_POSEE.publish(goal_msg)
        
    def letsee_current_pose(self):
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = rospy.Time.now()  
        current_pose_msg.header.frame_id = "odom"
        current_pose_msg.pose.position.x = self.current_x
        current_pose_msg.pose.position.y = self.current_y
        current_pose_msg.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.current_theta)
        current_pose_msg.pose.orientation.x = quaternion[0]
        current_pose_msg.pose.orientation.y = quaternion[1]
        current_pose_msg.pose.orientation.z = quaternion[2]
        current_pose_msg.pose.orientation.w = quaternion[3]
    
        self.CURRENT_POSE.publish(current_pose_msg)

    def draw_trajectory(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0
        self.trajectory_points.append(point)
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = self.trajectory_points
        self.TRAJ_PUB.publish(marker)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('CUSTOM_CAMERA_NAV')
    obs_avoi = ObstacleAvoidance()
    obs_avoi.run()
