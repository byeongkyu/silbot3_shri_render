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
import socket

from OpenGL import GL
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt, QUrl, pyqtSignal
# from PyQt5.QtWebEngine import QWebSettings
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
from PyQt5.QtNetwork import QSslConfiguration, QSsl

from std_msgs.msg import String
from mind_msgs.msg import RenderItemAction, RenderItemResult


class CustomWebEnginePage(QWebEnginePage):
    def certificateError(self, error):
        return True


class RenderScreenGui(QMainWindow):
    signalHandleView = pyqtSignal(str, str)
    signalHandleCommand = pyqtSignal(bool)

    def __init__(self):
        super(RenderScreenGui, self).__init__()
        self.view = QWebEngineView()
        self.setCentralWidget(self.view)
        self.setWindowFlags(Qt.FramelessWindowHint)

        self.view.loadFinished.connect(self.handle_load_finished)
        self.signalHandleView.connect(self.handle_view)
        self.signalHandleCommand.connect(self.handle_command)

        self.current_address = [ip for ip in socket.gethostbyname_ex(
            socket.gethostname())[2] if not ip.startswith("127.")][:1][0]
        self.current_port = rospy.get_param('/rosbridge_websocket/port', 9090)

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
        url_s = self.resource_path + '/mainView.html'
        if not self.use_remote:
            if not os.path.exists(url_s):
                url_s = self.resource_path + '/' + name + '.htm'
            if not os.path.exists(url_s):
                rospy.logerr('the file not exists. please check the file or resource path...')
                return

        address_str = urllib.quote('wss://' + self.current_address + ':' + str(self.current_port))
        screen_str = urllib.quote(screen)
        data_str = urllib.quote(data)

        page = CustomWebEnginePage(self.view)
        if not self.use_remote:
            page.load(QUrl("file://" + url_s + '?ros=' + address_str + '&data=' + data_str + '&name=' + screen_str))
        else:
            page.load(QUrl(url_s + '?ros=' + address_str + '&data=' + data_str + '&name=' + screen_str))
        self.view.setPage(page)

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
