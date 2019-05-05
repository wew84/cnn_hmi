#!/usr/bin/python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cnn_bridge.msg import Netmask, Detection
from tools import str2bool, Camera, DrawingTools

seg_draw = DrawingTools('segmentation')
dect_draw = DrawingTools('detection')
cam = None
mask = None
boxes = None
scores = None
classes = None

seq_img = 0.
seq_detect = 0.
seq_seg = 0.

sum_detect = [0.,0]
sum_seg = [0.,0]
def run_loop():
    global mask, sum_seg, sum_detect
    while not rospy.is_shutdown():
        frame, header = cam.get_image()
        seq_img = header.seq

        if mask is not None:
            frame = seg_draw.overlay_segmentation(frame, mask)
	    sum_seg[0] += seq_img - seq_seg
	    sum_seg[1] += 1

        if boxes is not None and scores is not None and classes is not None:
            frame = dect_draw.draw_detection(frame, boxes, scores, classes, [320,320])
	    sum_detect[0] += seq_img - seq_detect
            sum_detect[1] += 1
        frame_green = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        cv2.imshow('HMI', frame_green)
	rospy.logwarn((seq_img, seq_img - seq_detect, seq_img - seq_seg))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def segmentation_callback(data):
    global mask, seq_seg
    seq_seg = data.image_identifier.seq
    mask = np.reshape(data.mask.data,(data.mask.layout.dim[0].size, data.mask.layout.dim[1].size))

def detection_callback(data):
    global boxes, scores, classes, seq_detect
    seq_detect = data.image_identifier.seq
    boxes = np.array( [[x.x, x.y, x.z, x.w] for x in data.boxes])
    scores = [float(x.data) for x in data.scores]
    classes = [int(x.data) for x in data.classes]

def parse_args():
    """Parse the arguments provided when the module was initialized"""

    rospy.init_node('segmentation_publisher', log_level=rospy.DEBUG)
    params = {}
    #'The camera or video file to get the pictures from, '
    #          'options are: (String, Required) \n If video file, then '
    #          'path to the video file \n If ROS camera, then uEye, '
    #         'usb_cam \n If CV2 device, then device ID (0, 1, 3...)'
    if rospy.has_param('~source'):
        if isinstance(rospy.get_param('~source'), str):
            params['source'] = rospy.get_param('~source')
        else:
            raise rospy.ROSInitException(
                'The source display needs to be of type: String')
    else:
        raise rospy.ROSInitException('Param source is required (String)')

    # Path to KittiSeg Model Directory (String, Required)
    if rospy.has_param('~segmentation_topic'):
        if isinstance(rospy.get_param('~segmentation_topic'), str):
            params['segmentation_topic'] = rospy.get_param('~segmentation_topic')
        else:
            raise rospy.ROSInitException(
                'The segmentation_topic display needs to be of type: String')
    else:
        raise rospy.ROSInitException('Param segmentation_topic is required (String)')

    if rospy.has_param('~detection_topic'):
        if isinstance(rospy.get_param('~detection_topic'), str):
            params['detection_topic'] = rospy.get_param('~detection_topic')
        else:
            raise rospy.ROSInitException(
                'Thedetection_topic needs to be of type: String')
    else:
        raise rospy.ROSInitException('Param detection_topic is required (String)')

    return params


if __name__ == "__main__":
    args = parse_args()
    cam = Camera(args['source'])
    rospy.Subscriber(args['segmentation_topic'], Netmask, segmentation_callback, queue_size=1)
    rospy.Subscriber(args['detection_topic'], Detection, detection_callback, queue_size=1)
    run_loop()
    rospy.logwarn((sum_detect[0], sum_detect[1]))
    rospy.logwarn((sum_detect[0] / sum_detect[1], sum_seg[0] / sum_seg[1]))
