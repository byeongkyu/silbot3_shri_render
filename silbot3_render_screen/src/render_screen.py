#!/usr/bin/env python
#-*- encoding: utf8 -*-

import os
import sys
import signal
import rospy
import rospkg
import json
import actionlib
import urllib

from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt, QUrl, pyqtSignal
from PyQt5.QtWebKit import QWebSettings
from PyQt5.QtWebKitWidgets import QWebView
from PyQt5.QtNetwork import QSslConfiguration, QSsl

from std_msgs.msg import String
from mind_msgs.msg import RenderItemAction, RenderItemResult


class RenderScreenGui(QMainWindow):
    signalHandleView = pyqtSignal(str, str)
    signalHandleCommand = pyqtSignal(bool)

    def __init__(self):
        super(RenderScreenGui, self).__init__()
        self.view = QWebView()
        self.setCentralWidget(self.view)
        self.setWindowFlags(Qt.FramelessWindowHint)

        page = self.view.page()
        page.settings().setAttribute(QWebSettings.DeveloperExtrasEnabled, True)
        page.networkAccessManager().sslErrors.connect(self.sslErrorHandler)

        self.view.loadFinished.connect(self.handle_load_finished)
        self.signalHandleView.connect(self.handle_view)
        self.signalHandleCommand.connect(self.handle_command)

        try:
            self.resource_path = rospy.get_param('~resource_path')
        except KeyError:
            rospy.logerr('please set ~resource_path...')
            exit(-1)
        self.use_full_screen = rospy.get_param('~use_full_screen', False)

        self.use_remote = False
        if self.resource_path.startswith('https://') or self.resource_path.startswith('http://'):
            self.use_remote = True
        if not self.use_remote:
            self.resource_path = os.path.abspath(self.resource_path)
            self.resource_path = os.path.expanduser(self.resource_path)

        self.server = actionlib.SimpleActionServer(
            'render_screen', RenderItemAction, self.execute_callback, False)
        self.server.start()

        self.show()
        self.hide()
        rospy.loginfo('%s initialized...'%rospy.get_name())

    def sslErrorHandler(self, reply, errorList):
        url = unicode(reply.url().toString())
        reply.ignoreSslErrors()
        rospy.logwarn("SSL certificate error ignored: %s" % url)

    def handle_load_finished(self):
        rospy.logdebug('loading completed...')

    def handle_command(self, enable):
        if enable:
            if self.use_full_screen:
                self.showFullScreen()
            else:
                self.show()
        else:
            self.hide()

    def handle_view(self, name, data):
        url_s = self.resource_path + '/' + name + '.html'
        if not self.use_remote:
            if not os.path.exists(url_s):
                url_s = self.resource_path + '/' + name + '.htm'
            if not os.path.exists(url_s):
                rospy.logerr('the file not exists. please check the file or resource path...')
                return

        data_str = urllib.urlencode(json.loads(data))

        if not self.use_remote:
            self.view.load(QUrl("file://" + url_s + '?' + data_str))
        else:
            self.view.load(QUrl(url_s + '?' + data_str))


    def execute_callback(self, goal):
        result = RenderItemResult()
        success = True

        data = goal.data.split('/')
        if ':' in data[0]:
            screen_cmd, screen_name = data[0].split(':')
            screen_data = data[1]
        else:
            screen_cmd = data[0]
            screen_name = ''
            screen_data = '{}'

        if screen_cmd == 'clear':
            self.signalHandleCommand.emit(False)
        elif screen_cmd == 'show':
            self.signalHandleCommand.emit(True)
            self.signalHandleView.emit(screen_name, screen_data)

        rospy.sleep(0.2)
        if success:
            result.result = True
            self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('render_screen', anonymous=False)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    window = RenderScreenGui()

    # rospy.spin()
    sys.exit(app.exec_())
