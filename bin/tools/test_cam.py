import rospy 
import numpy as np
import timeit

COMPRESSED = False
if not COMPRESSED:
    from cv_bridge import CvBridge, CvBridgeError
    from sensor_msgs.msg import Image
else:
    import numpy as np
    import cv2
    from sensor_msgs.msg import CompressedImage


class testCam:
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted to cv2 format'''
        currentImage = ""
        if not COMPRESSED:
            currentImage =  self.bridge.imgmsg_to_cv2(ros_data, "rgb8")
        else:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, 1)

            currentImage = cv2.cvtColor(image_np, cv2.BGR2RGB)

        np.mean(currentImage)
        self.count += 1
        # if  self.count % 100 == 0:
            # rospy.logwarn( "testCam\t" + str(self.count) + ":\t" + 
            #                 str(float(self.count / (timeit.default_timer() -  self.start_time))))

    def __init__(self):
        self.bridge = CvBridge()
        self.count = 0
        self.start_time = timeit.default_timer()
        if not COMPRESSED:
            self.subscriber = rospy.Subscriber("ueye_0/image_raw",
                                                Image,  self.callback,  queue_size=1)
        else:
            sub_string += "/compressed"
            self.subscriber = rospy.Subscriber(
                sub_string, CompressedImage, callback, queue_size=1)
