#!/usr/bin/env python
#-*- coding: utf-8 -*-

import json
import rospy
import actionlib

from std_msgs.msg import String, Bool
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback


class Silbot3Screen:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('render_screen', RenderItemAction, self.execute_callback, False)
        self.server.start()
        rospy.loginfo('%s initialized...'%rospy.get_name())

    def execute_callback(self, goal):
        result = RenderItemResult()
        feedback = RenderItemFeedback()
        success = True

        data = goal.data.split('/')
        screen_cmd, screen_name = data[0].split(':')
        screen_data = json.loads(data[1])

        #
        # DO DISPLAY STUFF
        #

        print screen_cmd
        print screen_name
        print screen_data

        rospy.sleep(1)

        if success:
            result.result = True
            self.server.set_succeeded(result)

    def handle_ctrl_screen(self, msg):
        if msg.data:
            self.pub_screen.publish('show')
        else:
            self.pub_screen.publish('hide')


if __name__ == '__main__':
    rospy.init_node('silbot3_screen', anonymous=False)
    tts = Silbot3Screen()
    rospy.spin()
