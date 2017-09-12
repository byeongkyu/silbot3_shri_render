#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import yaml
import threading
import rospy
import json
import re
import actionlib
import random

from silbot3_msgs.srv import ExpressionStart
from silbot3_msgs.srv import ExpressionStartRequest
from silbot3_msgs.msg import ExpressionStatus, ExpressionSetModality

from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback
from mind_msgs.srv import GetInstalledGestures, GetInstalledGesturesResponse

class Silbot3Gesture:
    def __init__(self):
        self.startMotion = rospy.ServiceProxy('/silbot3_expression/start', ExpressionStart)
        self.stopMotion = rospy.ServiceProxy('/silbot3_expression/stop', ExpressionStart)
        self.subscriber = rospy.Subscriber('/silbot3_expression/status', ExpressionStatus, self.handle_expression_status)
        self.pub_set_modality = rospy.Publisher('/silbot3_expression/set_modality', ExpressionSetModality, queue_size=10)

        self.motionFinished = False
        self.lock = threading.RLock()

        self.startMotion.wait_for_service()
        self.stopMotion.wait_for_service()

        set_msg = ExpressionSetModality()
        set_msg.left_arm_enabled = True
        set_msg.right_arm_enabled = True
        set_msg.neck_enabled = False
        set_msg.wheel_enabled = False
        set_msg.face_enabled = False
        set_msg.led_enabled = False
        set_msg.sound_enabled = False
        self.pub_set_modality.publish(set_msg)
        rospy.sleep(1.0)

        try:
            motion_file = rospy.get_param('~motion_file')
        except KeyError:
            rospy.logerr('Set parammeter ~motion_file')
            quit(1)

        motion_file = os.path.abspath(motion_file)
        print motion_file
        stream = file(motion_file, 'r')
        self.motion_list = yaml.load(stream)

        self.getInstalledGesturesService = rospy.Service(
            "get_installed_gestures",
            GetInstalledGestures,
            self.getInstalledGestures
        )

        self.server = actionlib.SimpleActionServer('render_gesture', RenderItemAction, self.execute_callback, False)
        self.server.register_preempt_callback(self.stop_gesture)
        self.server.start()

        rospy.loginfo('%s initialized...'%rospy.get_name())

    def getInstalledGestures(self, request):
        result = json.dumps(self.motion_list)
        return GetInstalledGesturesResponse(result)

    def handle_expression_status(self, msg):
        rospy.sleep(0.1)
        if msg.expression_status == 0:
            self.motionFinished = True

    def execute_callback(self, goal):
        result = RenderItemResult()
        feedback = RenderItemFeedback()
        success = True

        gesture_type, gesture_data = goal.data.split('=')

        if gesture_type == 'gesture':
            (cmd, item_name) = gesture_data.split(':')
            if cmd == 'tag':
                match = re.search(r'\[(.+?)\]', item_name)
                rendering_gesture = ''
                if match:
                    item_name = item_name.replace(match.group(0), '')
                    emotion = match.group(1)
                    try:
                        rendering_gesture = self.motion_list[item_name][emotion][random.randrange(0, len(self.motion_list[item_name]) - 1)]
                    except (KeyError, TypeError):
                        rendering_gesture = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]
                else:
                    try:
                        rendering_gesture = self.motion_list[item_name][random.randint(0, len(self.motion_list[item_name]) - 1)]
                    except KeyError:
                        rendering_gesture = self.motion_list['neutral'][random.randint(0, len(self.motion_list[item_name]) - 1)]

                with self.lock:
                    if self.server.is_preempt_requested():
                        self.server.set_preempted()
                        rospy.logdebug("Gesture execution preempted before it started")
                        return

                    self.gesture = rendering_gesture
                    split_data = self.gesture.split('/')

                    req = ExpressionStartRequest()
                    req.expression_type = 2
                    req.package = split_data[0]
                    req.category = split_data[1]
                    req.id = split_data[2]
                    req.content = ''
                    res = self.startMotion(req)

                rospy.sleep(0.05)
                rospy.loginfo("Waiting for behavior execution to complete")
                while not self.motionFinished and not rospy.is_shutdown():
                    rospy.sleep(0.2)

                with self.lock:
                    self.gesture = None
                    succeed = True

            elif cmd == 'play':
                found_motion = False
                for k, v in self.motion_list.items():
                    if item_name in v:
                        found_motion = True

                if not found_motion:
                    error_msg = "Gesture '{}' not installed".format(item_name)
                    self.server.set_aborted(text = error_msg)
                    rospy.logerr(error_msg)
                    return
                else:
                    self.gesture = item_name
                    split_data = self.gesture.split('/')

                    req = ExpressionStartRequest()
                    req.expression_type = 0
                    req.package = split_data[0]
                    req.category = split_data[1]
                    req.id = split_data[2]
                    req.content = ''
                    res = self.startMotion(req)

                    rospy.sleep(0.05)
                    rospy.loginfo("Waiting for behavior execution to complete")
                    while not self.motionFinished and not rospy.is_shutdown():
                        rospy.sleep(0.2)

                    with self.lock:
                        self.gesture = None
                        succeed = True

        if success:            
            result.result = True
            self.motionFinished = False
            self.server.set_succeeded(result)

    def stop_gesture(self):
        with self.lock:
            if self.gesture and self.actionlibServer.is_active():
                # ? Stop gesture
                pass

if __name__ == '__main__':
    rospy.init_node('silbot3_facial_expression', anonymous=False)
    gesture = Silbot3Gesture()
    rospy.spin()
