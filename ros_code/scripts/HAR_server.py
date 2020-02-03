#!/usr/bin/env python3

from ha_recognition.srv import RecordActivity, RecordActivityResponse
from ha_recognition.msg import ActivityLabel
import rospy
from app_code.inference.opencv_camera import Webcam
from app_code.inference.HARController import HARController
from app_code.inference.imshow import ImageServer


class Recognition_server:

    def __init__(self, queue_size=10):
        self.queue_size = queue_size
        self.view = ImageServer()
        self.cam = Webcam(self.view.showImg)
        self.cam.start()
        self.har_ctrl = HARController(self.cam, self.view)
        self.har_ctrl.on_prediction = self.publish_prediction

    def record(self, req):
        start = rospy.get_time()
        self.har_ctrl.record(fps=25)
        print(f"rgb loop time: {rospy.get_time() - start}")
        return RecordActivityResponse(True)

    def recognition_server(self):
        rospy.init_node('ha_recognition')
        s = rospy.Service('ha_recognition', RecordActivity, self.record)
        self.pub = rospy.Publisher(
            'ha_classifier', ActivityLabel, queue_size=self.queue_size)
        print("ready to recognize")
        rospy.spin()
        self.cam.stop()

    def publish_prediction(self, prediction):
        self.pub.publish(ActivityLabel(prediction[0], prediction[1]))
        self.cam.start()

if __name__ == "__main__":
    server = Recognition_server(10)
    server.recognition_server()
