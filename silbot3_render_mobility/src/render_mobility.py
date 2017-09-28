#!/usr/bin/env python
#-*- encoding: utf8 -*-

import sys
import rospy
import actionlib
import tf
import math
import yaml

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty


class Silbot3Mobility:
    def __init__(self):
        try:
            waypoint_file = rospy.get_param('~waypoint_file')
        except KeyError:
            rospy.logerr('Set parammeter ~waypoint_file')
            quit(1)

        with open(waypoint_file) as f:
            self.waypoints = yaml.load(f)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.is_mobile_moving = False
        rospy.sleep(0.5)

        # if 'home1' in self.waypoints.keys():
        #     rospy.loginfo('Set initialpose to home in waypoint_file...')
        #
        #     msg = PoseWithCovarianceStamped()
        #     msg.header.stamp = rospy.Time.now()
        #     msg.header.frame_id = 'map'
        #
        #     msg.pose.pose.position.x = self.waypoints['home'][0]
        #     msg.pose.pose.position.y = self.waypoints['home'][1]
        #     msg.pose.pose.position.z = 0.0
        #
        #     quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.waypoints['home'][2])
        #
        #     msg.pose.pose.orientation.x = quat[0]
        #     msg.pose.pose.orientation.y = quat[1]
        #     msg.pose.pose.orientation.z = quat[2]
        #     msg.pose.pose.orientation.w = quat[3]
        #
        #     msg.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        #     msg.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        #     msg.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0
        #
        #     self.pub_initialpose.publish(msg)

        self.server = actionlib.SimpleActionServer('render_mobility', RenderItemAction, self.execute_callback, False)
        self.server.start()

        rospy.loginfo('%s initialized...'%rospy.get_name())

    def execute_callback(self, goal):
        result = RenderItemResult()
        feedback = RenderItemFeedback()
        success = True

        if ':' in goal.data:
            (move_cmd, target_waypoint) = goal.data.split(':')
        else:
            move_cmd = goal.data

        if move_cmd == 'move':
            rospy.loginfo('start moving base to %s...' % target_waypoint)

            move_goal = MoveBaseGoal()
            move_goal.target_pose.header.stamp = rospy.Time.now()
            move_goal.target_pose.header.frame_id = 'map'

            move_goal.target_pose.pose.position.x = self.waypoints[target_waypoint][0]
            move_goal.target_pose.pose.position.y = self.waypoints[target_waypoint][1]
            move_goal.target_pose.pose.position.z = 0.0

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.waypoints[target_waypoint][2])

            move_goal.target_pose.pose.orientation.x = quat[0]
            move_goal.target_pose.pose.orientation.y = quat[1]
            move_goal.target_pose.pose.orientation.z = quat[2]
            move_goal.target_pose.pose.orientation.w = quat[3]

            rospy.wait_for_service('/move_base/clear_costmaps')
            clear_map = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_map()

            self.client.send_goal(move_goal, done_cb=self.handle_moving_done, feedback_cb=self.handle_moving_feedback)
            self.is_mobile_moving = True

            while self.is_mobile_moving:
                if self.server.is_preempt_requested():
                    self.client.cancel_all_goals()
                    self.server.set_preempted()
                    rospy.loginfo('stop moving by request...')
                    return

                feedback.is_rendering = True
                self.server.publish_feedback(feedback)
                rospy.sleep(0.2)

            rospy.sleep(0.2)
            rospy.loginfo('complete moving base...')

        elif move_cmd is 'stop':
            rospy.loginfo('stop moving...')
            self.client.cancel_all_goals()

        if success:
            result.result = True
            self.server.set_succeeded(result)

    def handle_moving_done(self, state, result):
        self.is_mobile_moving = False

    def handle_moving_feedback(self, feedback):
        rospy.logdebug(feedback)



if __name__ == '__main__':
    rospy.init_node('silbot3_render_mobility', anonymous=False)
    gesture = Silbot3Mobility()
    rospy.spin()
