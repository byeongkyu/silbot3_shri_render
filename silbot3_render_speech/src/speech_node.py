#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import actionlib

import pyaudio
import wave
from ctypes import *
from contextlib import contextmanager

from silbot3_msgs.msg import TTSSetProperties
from silbot3_msgs.srv import TTSMake, TTSMakeRequest
from mind_msgs.msg import RenderItemAction, RenderItemResult, RenderItemFeedback

ERROR_HANDLER_FUNC = CFUNCTYPE(
    None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(file_name, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)


class Silbot3RenderSpeech:
    def __init__(self):
        # For Silbot3
        rospy.wait_for_service('/silbot3_tts/make')
        self.make_service_client = rospy.ServiceProxy('/silbot3_tts/make', TTSMake)
        rospy.wait_for_service('/tts/get_loggers')
        self.set_publisher = rospy.Publisher('/silbot3_tts/set_properties', TTSSetProperties, queue_size=10)

        rospy.sleep(0.5)

        self.server = actionlib.SimpleActionServer('render_speech', RenderItemAction, self.execute_callback, False)
        self.server.start()
        rospy.loginfo('%s initialized...'%rospy.get_name())

    def execute_callback(self, goal):
        rospy.loginfo('%s rendering requested [%s]...' % (rospy.get_name(), goal.data))

        result = RenderItemResult()
        feedback = RenderItemFeedback()
        result.result = True

        msg = TTSSetProperties()
        msg.speaker_id = 0 #SPEAKER_ID_ENGLISH_FEMALE=3, SPEAKER_ID_ENGLISH_MALE=4
        msg.speed = 100
        msg.volume = 100
        msg.pitch = 100
        self.set_publisher.publish(msg)

        filename = '/tmp/speech.wav'

        req = TTSMakeRequest()
        req.text = goal.data
        req.filepath = filename
        res = self.make_service_client(req)
        rospy.logdebug("make_tts : " + str(res))

        if res.tts_result != 0:
            return

        feedback.is_rendering = True
        self.server.publish_feedback(feedback)

        with noalsaerr():
            paudio = pyaudio.PyAudio()

        wave_file = wave.open(filename, 'rb')
        try:
            stream = paudio.open(format=paudio.get_format_from_width(wave_file.getsampwidth()),
                                 channels=wave_file.getnchannels(),
                                 rate=wave_file.getframerate(),
                                 output=True)
        except IOError:
            return

        chunk_size = 1024
        data = wave_file.readframes(chunk_size)
        while data != '':
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                result.result = False
                break

            stream.write(data)
            data = wave_file.readframes(chunk_size)
            self.server.publish_feedback(feedback)

        stream.stop_stream()
        stream.close()
        paudio.terminate()

        rospy.sleep(0.2)

        if result.result:
            self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('silbot3_render_speech', anonymous=False)
    tts = Silbot3RenderSpeech()
    rospy.spin()
