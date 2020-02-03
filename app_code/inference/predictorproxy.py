import os
import subprocess
import cv2
import threading


class PredictorProxy:
    def __init__(self,
                 cam_width,
                 cam_height,
                 shared_dir_path="/dev/shm/har_imgs"):
        self.cam_width = cam_width
        self.cam_height = cam_height
        self.shared_dir_path = shared_dir_path
        if not os.path.exists(self.shared_dir_path):
            os.makedirs(self.shared_dir_path)
        self.counter = 0
        self.start_event = threading.Event()
        threading.Thread(target=self.start_process).start()
        self.start_event.wait(40)
        print("STARTED<<<<<<<<<<<<<<<<<<<", flush=True)

    def _on_prediction(self, prediction):
        self.counter = 0
        self.on_prediction(prediction)

    def on_prediction(self, prediction):
        pass

    def start_process(self):
        predictor_ctrl = os.path.join(
            os.path.dirname(__file__), "predictorctrl.py")
        proc = subprocess.Popen(["python3", predictor_ctrl,
            self.shared_dir_path, str(self.cam_width), str(self.cam_height)],
            stdout=subprocess.PIPE)
        for line in iter(proc.stdout.readline, b''):
            line = str(line)[2:-1]
            if line.startswith(">>"):
                self.start_event.set()
            elif line.startswith("<<"):
                parsed = line[2:-2].split('~')
                confs = [float(x) for x in parsed[0].split("*")]
                labels = [str(x) for x in parsed[1].split("*")]
                self._on_prediction((confs, labels))
            else:
                print(line)

        proc.kill()
        proc.wait()
        del proc

    def process_and_predict(self, img):
        cv2.imwrite(f"{self.shared_dir_path}/{self.counter}.jpeg", img)
        self.counter += 1
        # print(f"[Proxy] frame number {self.counter}")
