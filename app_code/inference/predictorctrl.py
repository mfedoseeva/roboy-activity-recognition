import sys
import os
from pathlib import Path

p = str(Path(os.path.dirname(__file__)).parent.parent)
sys.path.insert(0, p)

from app_code.inference.predictor import Predictor, _SAMPLE_VIDEO_FRAMES
import opticalflow.opticalflow as opticalflow

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import cv2
import numpy as np
import time

import logging
import datetime

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')

time_counter = 0

def isCorrupted(img_path):
    with open(os.path.join(img_path), 'rb') as f:
        check_chars = f.read()[-2:]
        return check_chars != b'\xff\xd9'


class NewImageHandler(FileSystemEventHandler):
    def __init__(self, callback):
        self.callback = callback

    def load_and_call(self, src_path):
        if os.path.isdir(src_path):
            return
        self.callback(src_path)

    def on_created(self, event):
        # logging.info(f"on_created: {event.src_path}")
        self.load_and_call(event.src_path)

    def on_modified(self, event):
        # logging.info(f"on_modified: {event.src_path}")
        self.load_and_call(event.src_path)


class PredictorController:
    def __init__(self, shared_dir_path, cam_size):
        self.predictor = Predictor()

        self.clear()

        self.width, self.height = cam_size
        self.calc_dims()

        img_handler = NewImageHandler(self.process_and_predict)
        observer = Observer()
        observer.schedule(img_handler, shared_dir_path, recursive=False)
        observer.start()
        try:
            print(">>started", flush=True)
            observer.join()
        except:
            observer.stop()
            self.predictor.close()

    def calc_dims(self):
        self.calc_scaled_dims()
        self.calc_cropped_dims()

    def calc_scaled_dims(self, shortest=256):
        width, height = self.width, self.height
        if width < height:
            self.scaled_width = shortest
            self.scaled_height = int(round(height * self.scaled_width / width))
        else:
            self.scaled_height = shortest
            self.scaled_width = int(round(width * self.scaled_height / height))

    def calc_cropped_dims(self, side=224):
        x_0 = int((self.scaled_width - side) / 2)
        x_1 = x_0 + side
        y_0 = int((self.scaled_height - side) / 2)
        y_1 = y_0 + side
        self.cropped_point1 = (x_0, y_0)
        self.cropped_point2 = (x_1, y_1)

    def scale(self, image):
        return cv2.resize(image, (self.scaled_width, self.scaled_height),
                          interpolation=cv2.INTER_AREA)

    def crop(self, scaled_image):
        return scaled_image[self.cropped_point1[1]:self.cropped_point2[1],
                            self.cropped_point1[0]:self.cropped_point2[0]]

    def normalize(self, x):
        return (x.astype(np.float32) - 128) / 128

    def clear(self):
        self.prev = None
        self.curr = None
        self.rgbs = [None] * _SAMPLE_VIDEO_FRAMES
        self.flows = [None] * _SAMPLE_VIDEO_FRAMES

    def process_rgb(self, image):
        scaled = self.scale(image)
        cropped = self.crop(scaled)
        # cropped = cropped.astype(dtype=np.float32)
        return cropped

    def process_flow(self, flow, bound):
        flow = (flow + bound) * (255.0 / (2 * bound))
        flow = np.round(flow).astype(int)
        flow[flow >= 255] = 255
        flow[flow <= 0] = 0
        return flow

    def get_samples(self):
        rgb_sample = np.array(self.rgbs)
        rgb_sample = self.normalize(rgb_sample)
        rgb_sample = rgb_sample[np.newaxis, ]
        # (79, 224, 224, 2)
        flow_sample = np.array(self.flows)
        flow_sample = self.normalize(flow_sample)
        flow_sample = flow_sample[np.newaxis, ]
        return rgb_sample, flow_sample

    def predict(self):
        rgb_sample, flow_sample = self.get_samples()
        start_time = time.time()
        conf, label = self.predictor.predict(rgb_sample, flow_sample)
        end_time = time.time()
        logging.info(f"inference time: {end_time - start_time}")
        return conf, label

    def process_and_predict(self, img_path):
        # logging.info(f"Process image: {img_path}")
        idx = int(img_path.split('/')[-1].split('.')[0])
        # fixing on_modified event for frame 78 breaking the start of the new sample
        if (idx != 0 and self.rgbs[0] is None):
            return
        # logging.info(f"Image index: {idx}")
        if self.rgbs[idx] is not None:
            return

        global time_counter
        if time_counter == 0:
            time_counter = time.time()

        if isCorrupted(img_path):
            return
        img = cv2.imread(img_path)
        if img is None:
            return
        img = self.process_rgb(img)

        self.rgbs[idx] = img
        if self.prev is None:
            self.prev = cv2.cvtColor(self.rgbs[0], cv2.COLOR_BGR2GRAY)
        else:
            self.prev = self.curr
        self.curr = cv2.cvtColor(self.rgbs[idx], cv2.COLOR_BGR2GRAY)
        flow = opticalflow.optical_flow(self.prev, self.curr)
        flow = self.process_flow(flow, bound=20)
        self.flows[idx] = flow
        # logging.info(f"Done image: {img_path}")
        if idx == (_SAMPLE_VIDEO_FRAMES - 1):
            logging.info("Predicting")
            time_counter = time.time() - time_counter
            logging.info(f"processing_time: {time_counter}")
            time_counter = 0
            conf, label = self.predict()
            conf_str = str("*".join(map(str, conf)))
            label_str = str("*".join(map(str, label)))
            print("<<" + conf_str + "~" + label_str, flush=True)
            self.clear()


def usage():
    print("usage: python predictorctrl.py <path/to/shared/dir> "
          "<cam_width> <cam_height>")


if __name__ == '__main__':
    if len(sys.argv) < 4:
        usage()
        sys.exit(1)
    path = sys.argv[1]
    cam_size = (int(sys.argv[2]), int(sys.argv[3]))
    if not os.path.exists(path):
        print("Directory was not found")
        usage()

    if not os.path.isdir(path):
        print("Provided path is not a directory")
        usage()
    ctrl = PredictorController(path, cam_size)
