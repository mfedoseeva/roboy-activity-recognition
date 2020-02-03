#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def topic_face_callback(data):
    pass

if __name__ == '__main__':

    rospy.init_node('roboy_face')
    rospy.Subscriber(
        "/roboy/cognition/face/show_emotion", String, topic_face_callback)
    rospy.spin()
