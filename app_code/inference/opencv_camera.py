import cv2
import threading
from .camera import Camera


class Webcam(Camera):

    def __init__(self, showImg, idx=0, w=640, h=480,
                 exposure=0.008, iso=100):
        self.cap = cv2.VideoCapture(idx)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

        (self.ret, self.frame) = self.cap.read()
        self.dimensions = (self.cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                           self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Controllable exposure, doesn't work most of the time
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        # self.cap.set(cv2.CAP_PROP_ISO_SPEED, iso)
        self.showImg = showImg
        self.stopped = True
        self.display = False

    def _next(self):
        while not self.stopped:
            (self.ret, self.frame) = self.cap.read()
            if not self.stopped and self.display:
                self.showImg(self.frame)

    def start(self):
        self.stopped = False
        self.startDisplay()
        threading.Thread(target=self._next, args=()).start()

    def stop(self):
        self.stopped = True

    def startDisplay(self):
        self.display = True

    def stopDisplay(self):
        self.display = False

    def read(self):
        return self.ret, self.frame

    @property
    def width(self):
        return int(self.cap.get(3))

    @property
    def height(self):
        return int(self.cap.get(4))
