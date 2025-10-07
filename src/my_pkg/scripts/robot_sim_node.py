#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotSimulator:
    def __init__(self):
        rospy.init_node('robot_simulator_node', anonymous=True)
        self.robot_status = "Idle"
        self.status_publisher = rospy.Publisher('/robot_status', String, queue_size=10)
        self.cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.rate = rospy.Rate(10)
        rospy.loginfo("Robot Simulator node has started.")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        if linear_x > 0: self.robot_status = "Moving Forward"
        elif linear_x < 0: self.robot_status = "Moving Backward"
        elif angular_z > 0: self.robot_status = "Turning Left"
        elif angular_z < 0: self.robot_status = "Turning Right"
        else: self.robot_status = "Idle"
        rospy.loginfo(f"Received command. Status: {self.robot_status}")

    def run(self):
        while not rospy.is_shutdown():
            self.status_publisher.publish(self.robot_status)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_sim = RobotSimulator()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass