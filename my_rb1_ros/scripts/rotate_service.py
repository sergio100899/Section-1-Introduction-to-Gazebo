#! /usr/bin/env python

import rospy
import math
import tf
from my_rb1_ros.srv import Rotate, RotateResponse

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RotateRobot():

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('rotate_service_server')

        # Service Server
        self.service = rospy.Service('/rotate_robot', Rotate, self.rotate_request)

        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscriber to /odom topic
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Variables
        self.current_yaw = 0.0
        self.ctrl_c = False

        rospy.loginfo("Rotate Service is ready.")

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        rospy.loginfo("Apagando nodo, deteniendo el robot...")
        self.ctrl_c = True
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)  # Stops the robot

    def odom_callback(self, msg):
        """Callback function to update robot orientation from odometry."""
        orientation_q  = msg.pose.pose.orientation

        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        self.current_yaw = yaw

    def publish_once_in_cmd_vel(self, cmd):
        """Publishes a command in /cmd_vel ensuring there are subscribers."""
        while not self.ctrl_c:
            connections = self.cmd_vel_pub.get_num_connections()
            if connections > 0:
                self.cmd_vel_pub.publish(cmd)
                break
            else:
                self.rate.sleep()


    def rotate_request(self, request):
        """Handles the service request to rotate the robot."""
        rospy.loginfo(f"Requested rotation: {request.degrees} degrees")

        target_angle = math.radians(request.degrees)
        initial_yaw = self.current_yaw

        twist = Twist()

        twist.angular.z = 0.2 if request.degrees > 0 else -0.2

        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(20)  # Rotation timeout

        response = RotateResponse()

        while not self.ctrl_c:
            rotated_angle = abs(self.current_yaw - initial_yaw )

            if rotated_angle >= abs(target_angle):  # Stop when reaching target
                rospy.loginfo("Rotation successfully completed.")
                response.result = f"Rotation of {request.degrees} degrees succesfully completed."
                break   

            if rospy.Time.now() > timeout:
                rospy.logwarn("Rotation was not completed within the expected time.")
                response.result = f"Rotation of {request.degrees} degrees not completed."
                break
           

            self.publish_once_in_cmd_vel(twist)
            rate.sleep()

        # Stop the robot
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        
        rospy.loginfo(response.result)

        return response

if __name__ == "__main__":
    RotateRobot()
    rospy.spin()
