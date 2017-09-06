#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import actionlib

from silbot3_msgs.msg import  Device_LED_Msg
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback

class Silbot3FacialExpression:

    led_color = {}
    led_color['neutral']    = (1.0,  1.0,  1.0)
    led_color['happiness']  = (0, 1, 0)
    led_color['surprise']   = (1, 1, 0)
    led_color['anger']      = (1, 0, 0)
    led_color['sadness']    = (0, 0, 1)
    led_color['disgust']    = (1, 0, 1)
    led_color['fear']       = (1, 0, 1)
    led_color['sleepiness'] = (1, 1, 0)

    def __init__(self):
        self.led_publisher = rospy.Publisher("/DeviceNode/LED/commands", Device_LED_Msg, queue_size=10)

        self.server = actionlib.SimpleActionServer('render_facial_expression', RenderItemAction, self.execute_callback, False)
        self.server.start()
        rospy.loginfo('%s initialized...'%rospy.get_name())

    def execute_callback(self, goal):
        result = RenderItemResult()
        feedback = RenderItemFeedback()
        success = True

        rospy.sleep(0.2)

        led_msg = Device_LED_Msg()
        led_msg.command = "on"
        led_msg.id = 0xF5
        led_msg.bright = 1
        led_msg.red = 128 #self.led_color[goal.data][0]
        led_msg.green = self.led_color[goal.data][1]
        led_msg.blue = self.led_color[goal.data][2]

        self.led_publisher.publish(led_msg)
        rospy.sleep(1)

        # Facial Expression


        if success:
            result.result = True
            self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('silbot3_facial_expression', anonymous=False)
    tts = Silbot3FacialExpression()
    rospy.spin()
