#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class RevolvingTurtle:

    def __init__(self):

        rospy.init_node('node_turtle_revolve', anonymous=False)

        self.pub_topic = '/turtle1/cmd_vel'
        self.velocity_publisher = rospy.Publisher(self.pub_topic,
                                            Twist, queue_size=10)
        self.sub_topic = '/turtle1/pose'
        self.pose_subscriber = rospy.Subscriber(self.sub_topic,
                                        Pose, self.update_pose)
        self.pose = Pose()
        self.twist = Twist()
        self.rate = rospy.Rate(10)

    def update_pose(self, msg):
        self.pose = msg
        self.pose.x = round(msg.x, 4)
        self.pose.y = round(msg.y, 4)
        self.pose.theta = round(msg.theta, 4)

    def dist_from_init(self, init_time):
        final_time = float(rospy.Time.now().to_sec())
        return self.twist.linear.x * (final_time - init_time - 0.2)

    def revolve(self):
        freq = 0.5
        radius = 2

        angular_speed = 2 * math.pi * freq
        linear_speed = angular_speed * radius

        init_time = float(rospy.Time.now().to_sec())
        distance = 0
        circumference = 2 * math.pi * radius
        rospy.loginfo('Circumference = ' + str(round(circumference, 2)))

        self.twist.linear.x = round(linear_speed, 4)
        self.twist.linear.y = 0
        self.twist.linear.z = 0

        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = round(angular_speed, 4)

        while distance <= circumference :
            distance = self.dist_from_init(init_time)
            rospy.loginfo('Revolving...\n' + 'Distance = '
                                + str(round(distance, 5)))
            self.velocity_publisher.publish(self.twist)
            self.rate.sleep()

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.velocity_publisher.publish(self.twist)

        rospy.spin()

if __name__ == '__main__':
    try:
        revolving_turtle = RevolvingTurtle()
        revolving_turtle.revolve()

    except rospy.ROSInterruptException:
        rospy.loginfo('Error encountered. node_turtle_revolve terminated.')
