#!/usr/bin/env python3

from ha_recognition.srv import RecordActivity, RecordActivityResponse
from ha_recognition.msg import ActivityLabel
import rospy
import time
import threading


class Recognition_server:

    def __init__(self, queue_size=10):
        self.queue_size = queue_size

    def record(self, req):
        processing_thread = threading.Thread(target=self.process)
        processing_thread.start()
        time.sleep(3)
        return RecordActivityResponse(True)

    def recognition_server(self):
        rospy.init_node('ha_recognition')
        s = rospy.Service('ha_recognition', RecordActivity, self.record)
        self.pub = rospy.Publisher(
            'ha_classifier', ActivityLabel, queue_size=self.queue_size)
        print("ready to recognize")
        rospy.spin()

    def process(self):
        start = rospy.get_time()
        print(start)
        time.sleep(7)
        conf = [0.78, 0.91, 0.3]
        boop_str = ["boop", "foo", "bar"]
        self.pub.publish(ActivityLabel(conf, boop_str))
        print(start - rospy.get_time())


if __name__ == "__main__":
    server = Recognition_server(10)
    server.recognition_server()
