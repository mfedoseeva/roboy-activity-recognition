#!/usr/bin/env python

import rospy
from ha_recognition.srv import RecordActivity
from ha_recognition.msg import ActivityLabel
from talk_client import talk_client

def recognition_client():
    rospy.wait_for_service("ha_recognition")
    try:
        record = rospy.ServiceProxy("ha_recognition", RecordActivity)
        resp = record()
        return resp
    except rospy.ServiceException as e:
        print('Service call failed: ', e)

def listener_callback(label):
    print(label.label[0])
    for x, y in zip(label.confidence, label.label):
        print(x, y)
    recognition_client()

def listener():
    rospy.init_node("HAR_client")
    rospy.Subscriber("ha_classifier", ActivityLabel, listener_callback)
    rospy.spin()

if __name__ == '__main__':
    if(recognition_client()):
        listener()
