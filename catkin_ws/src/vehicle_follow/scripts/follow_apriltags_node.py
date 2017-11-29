#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped, Point

class AprilFollow(object):

    def __init__(self):    
        self.node_name = "follow_apriltags_node"
        self.pose = Point()

        # -------- subscriber --------
        self.sub_pose = rospy.Subscriber("~position_info", Point, self.callback, queue_size=1)

        # -------- publisher --------
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        print ("Start to follow object:")

    def callback(self, msg):
        self.pose = msg
        self.car_cmd(msg)
        self.stop()
        
    def car_cmd(self, pose):
        cmd = Twist2DStamped()
        #cmd.v = self.kp*(pose.z-self.goal_z)+self.kd*()
        # Calculate Latency up to this point starting from camera image
        delta_t = (rospy.Time.now() - pose.header.stamp).to_sec()  # latency
        #header.stamp = rospy.Time.now()

        # decide if smith controller is used or not by variable self.alpha
        delta_t_alpha = self.alpha * delta_t
        [delta_omega, delta_x, delta_y] = self.integrate(self.last_car_cmd_msg.omega, self.last_car_cmd_msg.v, delta_t_alpha)

        # Calculate the actual deltas in position:
        actual_x = pose.x - delta_x
        actual_y = pose.y - delta_y
        delta_dist_vec = np.array([actual_x, actual_y])

        # Calculate distance rho for polar coordinates:
        actual_rho = np.linalg.norm(delta_dist_vec)

        # Calculate angle theta for vehicle end pose:
        actual_theta = pose.theta - delta_omega


        # WRITE new car command
        # take over header of message
        self.car_cmd_msg.header = pose.header

        # Following Error Calculation
        following_error = actual_rho - self.dist_ref
        self.car_cmd_msg.v = self.k_follow * following_error

        # Clipping of velocity control effort:
        if self.car_cmd_msg.v > self.max_speed:
            self.car_cmd_msg.v = self.car_cmd_msg.v
        if self.car_cmd_msg.v < - self.max_speed:
            self.car_cmd_msg.v = - self.max_speed
        # Dead space of velocity control effort:
        elif abs(self.car_cmd_msg.v) < self.deadspace_speed:
            self.car_cmd_msg.v = 0.0

        # Heading Error Calculation
        # Combining heading error with target vehicle psi
        heading_error = actual_theta - self.head_ref + pose.psi * self.alpha_psi

        self.car_cmd_msg.omega = self.k_heading * heading_error

        if self.car_cmd_msg.omega > self.max_heading:
            self.car_cmd_msg.omega = self.max_heading
        elif self.car_cmd_msg.omega < -self.max_heading:
            self.car_cmd_msg.omega = -self.max_heading
        elif abs(self.car_cmd_msg.omega) < self.deadspace_heading:
            self.car_cmd_msg.omega = 0.0

        self.last_omega = self.car_cmd_msg.omega
        self.last_v = self.car_cmd_msg.v

        # Publish control message
        self.pub_car_cmd.publish(self.car_cmd_msg)

        self.last_vehicle_pose = pose
        self.last_car_cmd_msg = self.car_cmd_msg

    def integrate(self, theta_dot, v, dt):
        theta_delta = theta_dot * dt
        if abs(theta_dot) < 0.000001:  # to ensure no division by zero for radius calculation
            # straight line
            x_delta = v * dt
            y_delta = 0
        else:
            # arc of circle, see "Probabilitic robotics"
            radius = v / theta_dot
            x_delta = radius * np.sin(theta_delta)
            y_delta = radius * (1.0 - np.cos(theta_delta))
        return [theta_delta, x_delta, y_delta]
    def stop(self):
        rospy.sleep(0.3)
        cmd = Twist2DStamped()
        cmd.v = 0
        cmd.omega = 0
        self.pub_car_cmd.publish(cmd)

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilFollow()
    rospy.spin()
