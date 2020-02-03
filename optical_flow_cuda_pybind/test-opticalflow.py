import sys
sys.path.insert(0, '../')
import opticalflow.opticalflow as opticalflow
import cv2
import numpy as np

import time
millis = lambda: int(round(time.time() * 1000))

img1_path = "./app_code/optical_flow_cuda_pybind/basketball1.png";
img2_path = "./app_code/optical_flow_cuda_pybind/basketball2.png";

prev = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)
curr = cv2.imread(img2_path, cv2.IMREAD_GRAYSCALE)

if prev is None or curr is None:
	print("images not found")

tick = millis()
flow_gpu = opticalflow.optical_flow(prev, curr)
assert flow_gpu.dtype == np.float32
print("GPU time:", millis() - tick)

tick = millis()
TVL1 = cv2.optflow.DualTVL1OpticalFlow_create()
flow = TVL1.calc(prev, curr, None)
print("CPU time:", millis() - tick)

def save_flow(flow, name, bound=20):
    flow = (flow + bound) * (255.0 / (2*bound))
    flow = np.round(flow).astype(int)
    flow[flow >= 255] = 255
    flow[flow <= 0] = 0

    cv2.imwrite(f"{name}_0.jpg", flow[:, :, 0])
    cv2.imwrite(f"{name}_1.jpg", flow[:, :, 1])

save_flow(flow_gpu, "gpu")
save_flow(flow, "cpu")
