from .predictorproxy import PredictorProxy
from .predictor import _SAMPLE_VIDEO_FRAMES
import time
import cv2
import os

import datetime


_DEFAULT_IMG = cv2.imread(os.path.join(
    os.path.dirname(__file__), 'templates', 'Roboy_chef.jpg'))

if _DEFAULT_IMG is None:
    raise ValueError("Default image was not found")

class HARController:
    def __init__(self, camera, view):
        self.cam = camera
        self.view = view
        self.predictor = PredictorProxy(self.cam.width, self.cam.height)
        self.predictor.on_prediction = self._on_prediction

    def _on_prediction(self, prediction):
        self.view.current_text = ""
        self.on_prediction(prediction)

    def on_prediction(prediction):
        pass

    def record(self, fps=25):
        self.view.current_text = 'RECORDING...'
        self.view.progressStart()
        delay = 1 / fps  # in seconds
        counter = 0
        # loop_start = time.time()
        while counter < _SAMPLE_VIDEO_FRAMES:
            ts = time.time()
            _, frame = self.cam.read()
            self.predictor.process_and_predict(frame)
            counter += 1
            ts = time.time() - ts
            try:
                time.sleep(delay - ts)
            except:
                break
        # rgb_time = time.time() - loop_start
        self.cam.stop()
        self.view.current_text = 'Thinking'
        self.view.showImg(_DEFAULT_IMG)
