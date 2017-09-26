#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
import actionlib

from mind_msgs.msg import RenderItemAction, RenderItemGoal


def func_done(state, result):
    print 'Done'


def func_active():
    print "Start"


def main():
    client = actionlib.SimpleActionClient('render_mobility', RenderItemAction)
    client.wait_for_server()

    goal = RenderItemGoal(name='mobility', data='move:home')
    client.send_goal(goal, done_cb=func_done, active_cb=func_active)

    rospy.sleep(2)
    goal = RenderItemGoal(name='mobility', data='stop')
    client.send_goal(goal, done_cb=func_done, active_cb=func_active)
    client.cancel_all_goals()
    # client.wait_for_result()
    rospy.signal_shutdown(0)


if __name__ == '__main__':
    rospy.init_node('test_mobility_node', anonymous=False)
    m = main()
    rospy.spin()
