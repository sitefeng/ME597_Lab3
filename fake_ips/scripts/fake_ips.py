#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
import time


from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path


class Converter():


    def __init__(self):
        rospy.init_node('fake_ips')
        

        self.out =PoseWithCovarianceStamped()

        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.path_pub_index = 0
        self.path_pub_rate = 10000


        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.pub = rospy.Publisher("/indoor_pos", PoseWithCovarianceStamped, queue_size=10)

        self.path_pub = rospy.Publisher("/robot_path", Path, queue_size=5)

    def callback(self, msg):
        for n, p, t in zip(msg.name, msg.pose, msg.twist):
            #print(n)
            if n == 'mobile_base':
                self.out.pose.pose = p
                #self.out.orientation = t

                self.pub.publish(self.out)


                ps = PoseStamped()
                ps.pose = p
                

                self.path_pub_index = (self.path_pub_index + 1) % self.path_pub_rate
                if self.path_pub_index == 0:
                    self.path.poses.append(ps)
                    self.path_pub.publish(self.path)


    def main(self):

        while not rospy.is_shutdown():
            time.sleep(1)


if __name__ == "__main__":

    c = Converter();

    c.main()