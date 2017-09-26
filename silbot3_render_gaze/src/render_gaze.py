#!/usr/bin/env python
#-*- coding: utf-8 -*-

import sys
import rospy
import tf2_ros
import numpy as np
import threading
import math

from mind_msgs.msg import GazeCommand
from tf2_geometry_msgs import PointStamped
# from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, Bool


class GazeRenderNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.tf_buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buf)

        rospy.Subscriber('gaze_command', GazeCommand, self.handle_gaze_point)
        self.pub_pan = rospy.Publisher('head_pan/command', Float64, queue_size=10)
        self.pub_tilt = rospy.Publisher('head_tilt/command', Float64, queue_size=10)
        self.pub_xtion_tilt = rospy.Publisher('xtion_tilt/command', Float64, queue_size=10)
        self.pub_pan_max_speed = rospy.Publisher('head_pan_max_speed/command', Float64, queue_size=10)
        self.pub_tilt_max_speed = rospy.Publisher('head_tilt_max_speed/command', Float64, queue_size=10)

        with self.lock:
            self.enable_gaze = True
        rospy.Subscriber('0_enable_gaze', Bool, self.handle_enable_gaze)
        rospy.sleep(0.5)

        self.pub_pan_max_speed.publish(40.0)
        self.pub_tilt_max_speed.publish(40.0)
        self.pub_xtion_tilt.publish(0.46)

        with self.lock:
            self.target = GazeCommand()
            self.target.target_point.header.frame_id = "base_footprint"
            self.target.target_point.point.x = 2.0
            self.target.target_point.point.y = 0.0
            self.target.target_point.point.z = 1.0
            self.target.max_speed = 0.1

        rospy.Timer(rospy.Duration(0.1), self.handle_gaze_controller)
        rospy.loginfo('[%s] initialzed...' % rospy.get_name())

    def handle_enable_gaze(self, msg):
        with self.lock:
            self.enable_gaze = msg.data

    def handle_gaze_controller(self, event):
        with self.lock:
            try:
                aaa = PointStamped()
                aaa.header.stamp = rospy.Time()
                aaa.header.frame_id = self.target.target_point.header.frame_id
                aaa.point.x = self.target.target_point.point.x
                aaa.point.y = self.target.target_point.point.y
                aaa.point.z = self.target.target_point.point.z
                point_transformed = self.tf_buf.transform(aaa, 'head_base')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                e = sys.exc_info()[0]
                rospy.logdebug(e)
                rospy.logwarn("[%s] Can't tranform from gaze to target.[ %s - %s ]"%(rospy.get_name(), 'gaze', self.target.target_point.header.frame_id))
                return

        pan_angle = math.atan2(point_transformed.point.y, point_transformed.point.x)
        tilt_angle = math.atan2(point_transformed.point.z, point_transformed.point.x)

        # print pan_angle * 180 / math.pi, tilt_angle * 180 / math.pi

        if self.enable_gaze:            
            self.pub_pan.publish(pan_angle)
            self.pub_tilt.publish(tilt_angle)

    def handle_gaze_point(self, msg):
        with self.lock:
            self.target = msg


if __name__ == '__main__':
    rospy.init_node('silbot3_render_gaze', anonymous=False)
    m = GazeRenderNode()
    rospy.spin()
