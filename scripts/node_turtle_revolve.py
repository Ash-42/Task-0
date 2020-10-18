#!/usr/bin/env python

""" This script implements a node that causes the
circular motion of turtlesim in the given plane of
circulation until the turtle completes one revolution """

import math
import rospy
from geometry_msgs.msg import Twist # Twist message with linear and angular components of velocity
from turtlesim.msg import Pose      # Pose message with position and angular coordinates

class RevolvingTurtle:

    """ Initializes a node node_turtle_revolve that publishes defined
    linear and angular velocities to the turtle at a rate of 10 Hz """

    def __init__(self):

        # Initialize the ROS node
        rospy.init_node('node_turtle_revolve', anonymous=True)

        # Initialize publisher for the Twist message from the /turtle1/cmd_vel topic
        self.pub_topic = '/turtle1/cmd_vel'
        self.velocity_publisher = rospy.Publisher(self.pub_topic,
                                                  Twist, queue_size=10)
        # Initialize subscriber for the Pose message from the /turtle1/pose topic
        self.sub_topic = '/turtle1/pose'
        self.pose_subscriber = rospy.Subscriber(self.sub_topic,
                                                Pose, self.update_pose)
        # Set the pose and twist attributes
        self.pose = Pose()
        self.twist = Twist()
        # Maintains a loop at 10 Hz
        self.rate = rospy.Rate(10)

    def update_pose(self, msg):

        """ Callback function that updates the position and angle
        of the node's attribute with the turtle's pose message"""

        self.pose = msg
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = msg.theta

    def dist_from_init(self, init_time):

        """ Calculates the distance traveled by
        the turtle since the start of execution """

        # Captures the present time
        final_time = float(rospy.Time.now().to_sec())
        # Sets the time interval of motion
        bias = 0.2
        time_taken = final_time - init_time
        if time_taken > bias:
            time_taken -= bias
        # Distance is linear speed times the time of circular motion
        return self.twist.linear.x * time_taken

    def revolve(self):

        """ Causes circular motion of the turtle by publishing
        the twist message to the cmd_vel topic at the rate of 10 Hz """

        # Set values for the radius and frequecy of circular motion
        radius = 2
        frequency = 0.5

        # Calculate linear and angular speeds
        angular_speed = 2 * math.pi * frequency
        linear_speed = angular_speed * radius

        # Update the linear velocity component of the twist message
        self.twist.linear.x = linear_speed
        self.twist.linear.y = 0
        self.twist.linear.z = 0

        # Update the angular velocity component of the twist message
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = angular_speed

        # Initialize the distance traveled and circumference
        distance = 0
        circumference = 2 * math.pi * radius
        rospy.loginfo('Circumference = ' + str(round(circumference, 7)))

        # Capture the time at the start of circular motion
        init_time = float(rospy.Time.now().to_sec())

        # Publishing loop
        while 1:
            distance = self.dist_from_init(init_time)
            # Publish twist message until one revolution is traversed
            if distance < circumference:
                rospy.loginfo('Revolving...\n' + 'Distance = '
                              + str(round(distance, 7)))
                self.velocity_publisher.publish(self.twist)
                self.rate.sleep()
            else:
                rospy.loginfo('Completed one revolution')
                break

        # Reset twist message and stop turtle motion
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.velocity_publisher.publish(self.twist)

        # Keeps node from exiting till shutdown
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create node instance and call revolve function
        REV_TURTLE = RevolvingTurtle()
        REV_TURTLE.revolve()

    except rospy.ROSInterruptException:
        rospy.loginfo('Error encountered. node_turtle_revolve terminated.')
