from .camera import Camera
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ZedCamera(Camera):

    def __init__(self, showImg):
        self.showImg = showImg
        self.bridge = CvBridge()
        self.stopped = True
        self.ret = False
        rospy.init_node('zed_listener', anonymous=True)
        self.camera_info = rospy.wait_for_message("/zed/zed_node/rgb/camera_info", CameraInfo)
        self.width = CameraInfo.width
        self.height = CameraInfo.height


    def _next(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.ret = True
            if not self.stopped:
                self.showImg(self.frame)
        except CvBridgeError as e:
            print(e)

    def start(self):
        self.stopped = False
        rospy.Subscriber("/zed/zed_node/rgb/image_raw_color", Image, self._next)
        # TODO
        # rospy.spin() figure out how to stop

    def stop(self):
        self.stopped = True

    def read(self):
        return self.ret, self.frame

    @property
    def width(self):
        return self.width

    @property
    def height(self):
        return self.height