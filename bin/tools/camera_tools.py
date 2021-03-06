#!/usr/bin/python
"""
The Camera module initialized and provides images from cv2 and ROS cameras

Input: Camera type and parameters
Output: Images

Submodule of the cnn_publisher program

--------------------------------------------------------------------------------

Copyright (c) 2018 Perfetto Team, Technical Division, Israel Defence Force

"""
from os import mkdir, path
import sys
import datetime
import cv2

import rospy
from std_msgs.msg import Header

import timeit
from test_cam import testCam

class Camera(object):
    """Class defining a camera"""
    def __init__(self, source, save_images=False):
        self.original_image_size = None
        self.show_size_ratio = None
        self.image_size_to_show = None
        self.camera_type = None
        self.video_source = None
        self.frame_counter = None
        self.a = testCam()
        self.count = 0
        self.delay = 0
        # Setup ROS and Camera
        if source.lower() == 'ueye' or source.lower() == 'usb_cam' or source.lower() == 'cv_cam':
            self.setup_ros_cam(source)
        else:
            self.setup_cv2_cam(source)

        if save_images:
            self.setup_image_saving()

    def __del__(self):
        if "cv2" in self.camera_type and self.video_source.isOpened():
            self.video_source.release()

    def setup_ros_cam(self, source):
        """Set up a ROS camera
        Input:
            source  (str) The camera to setup
        Output: None"""
        try:
            from image_subscriber import ImageSubscriber
        except ImportError:
            # You forgot to initialize submodules
            rospy.loginfo("Could not import the submodules.")

            import traceback
            ex_type, ex, tb = sys.exc_info()
            traceback.print_tb(tb)

            exit(1)

        self.camera_type = "ROS"
        self.video_source = ImageSubscriber(cam_type=source)
        self.set_image_parameters(self.get_first_ros_frame())

    def setup_cv2_cam(self, source):
        """Set up a cv2 camera
        Input:
            source (str) The camera to setup
        Output: None"""
        try:
            warning_prefix = '\n\t__Warning__: \t Video '
            self.camera_type = "cv2cam"
            source = int(source)
            if not path.exists('/dev/video%d' % source):
                rospy.loginfo(warning_prefix + 'stream /dev/video%d is absent\n' %
                             source)

        except ValueError:
            self.camera_type = "cv2vid"
            if not path.exists(source):
                rospy.loginfo(warning_prefix +
                             'file %s is not found\n' % source)
                raise Exception("Input Video " + source + " is absent")

        self.video_source = cv2.VideoCapture(source)

        self.check_cv2_camera(source)

        self.frame_counter = 0
        self.frame_count = self.find_frame_counter(source)
        self.set_image_parameters(self.get_image()[0])

    def check_cv2_camera(self, source):
        """Verify that a cv2 camera is opened
        Input:
            source (str) The camera to setup
        Output: None"""
        # Check if video source is open
        if not self.video_source.isOpened():
            raise Exception("Can not open video " + source)

    def find_frame_counter(self, source):
        """Find the frame count of a video
        Input:
            source (str) The cv2 source
        Output: (int or None)
            The number of frames"""
        if isinstance(source, str):
            rospy.logdebug('Video %s opened  with %d frames' %
                          (source, self.video_source.get(cv2.CAP_PROP_FRAME_COUNT)))
            return int(self.video_source.get(cv2.CAP_PROP_FRAME_COUNT))
        return None

    def change_cv2_resolution(self):
        """Change the resolution of a cv2 device to match the input size
        Input: None
        Output: None"""
        # Try to change camera resolution to 800x600
        self.video_source.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.video_source.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        # h_video_in.set(cv2.CAP_PROP_FOCUS,10)
        self.original_image_size = (
            self.video_source.get(cv2.CAP_PROP_FRAME_WIDTH),
            self.video_source.get(cv2.CAP_PROP_FRAME_HEIGHT))  # float
        rospy.logdebug('Try to change resolution. Get:' +
                      str(self.original_image_size))

    # def delay_calc(self, start_time):
    #     self.count += 1 
    #     self.delay += float((timeit.default_timer() - start_time))
    #     if self.count % 100 == 0 and (self.camera_type == "ROS" and self.video_source.currentImage is not None):
    #         rospy.logwarn( "Delay "+ str(self.count) +":\t" + str(float(self.delay / self.count)))


    def get_image(self):
        """Get a single image
        Input: None
        Output: cv2 image in RGB"""
        start_time = timeit.default_timer()
        
        if self.camera_type == 'ROS':
            retn = self.video_source.currentImage, self.video_source.currentHeader
            # self.delay_calc(start_time)
            return retn

        ret, frame = self.video_source.read()

        header = Header()
        header.seq = self.frame_counter
        header.stamp = rospy.Time.now()
        header.frame_id = self.camera_type

        # self.delay_calc(start_time) 
        if ret:
            self.frame_counter += 1
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), header
        else:
            return None, None

    def get_first_ros_frame(self):
        """Wait for the first frame of a ROS camera.
        Due to issues with the initialization sometimes it takes
        several calls to get the first image

        Input: None
        Output: cv2 image in BGR"""
        first_image = self.get_image()[0]
        while first_image is None and not rospy.is_shutdown():
            first_image = self.get_image()[0]
        return first_image

    def set_image_parameters(self, image):
        """Set the size parameters of the image
        Input:
            image (cv2 Image) The image to take the sizes from
        Output: None"""
        self.original_image_size = (image.shape[1],
                                    image.shape[0])  # float
        self.show_size_ratio = 1000.0 / self.original_image_size[0]
        print(self.show_size_ratio)
        self.image_size_to_show = (int(self.original_image_size[0] * self.show_size_ratio),
                                   int(self.original_image_size[1] * self.show_size_ratio))

    def get_from_camera_video_name(self):
        """Create name for output video if source is a camera"""
        #Create default name
        if self.camera_type is not 'cv2vid':
            return 'Camera_' + self.camera_type + '__' + \
                datetime.datetime.now().strftime("%Y-%m-%d__%H:%M:%S")
        else:
            return ''

    def setup_image_saving(self):
        """Set up the ability to save all the capture images
        Input: None
        Output: None"""
        base_dir_name = self.camera_type + datetime.datetime.now().strftime("%Y-%m-%d__%H:%M:%S")

        if path.exists(base_dir_name + '_org') or path.exists(base_dir_name + '_grn'):
            add_ind = 0
            dir_name = base_dir_name
            while path.exists(dir_name + '_org') or path.exists(dir_name + '_grn'):
                add_ind += 1
                dir_name = base_dir_name + \
                    '_%d' % add_ind
        else:
            dir_name = base_dir_name

        mkdir(dir_name + '_org')
        mkdir(dir_name + '_grn')
        rospy.loginfo('Folders for images ' + dir_name + '_org' +
                     ' and ' + dir_name + '_grn' + ' were created')
