#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import actionlib

from std_msgs.msg import String, Bool
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback

class Silbot3Screen:
    def __init__(self):
        self.pub_screen = rospy.Publisher('/silbot3_display/page/show', String, queue_size=10)
        self.sub_ctrl_screen = rospy.Subscriber('/render_screen/onoff', Bool, self.handle_ctrl_screen)

        # Default off
        rospy.sleep(0.5)
        self.pub_screen.publish('hide')

        self.server = actionlib.SimpleActionServer('render_screen', RenderItemAction, self.execute_callback, False)
        self.server.start()
        rospy.loginfo('%s initialized...'%rospy.get_name())

    def execute_callback(self, goal):
        result = RenderItemResult()
        feedback = RenderItemFeedback()
        success = True

        self.pub_screen.publish('show')
        rospy.sleep(0.5)

        self.pub_screen.publish(goal.data)
        rospy.sleep(0.5)

        if success:
            result.result = True
            self.server.set_succeeded(result)

    def handle_ctrl_screen(self, msg):
        if msg.data:
            self.pub_screen.publish('show')
        else:
            self.pub_screen.publish('hide')


if __name__ == '__main__':
    rospy.init_node('silbot3_facial_expression', anonymous=False)
    tts = Silbot3Screen()
    rospy.spin()
