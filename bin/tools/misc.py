#!/usr/bin/python
import rospy
import roslib
import cv2

def exit_handler(camera, video_out, statistics_engine):
    """Hadle when the program exits. calculate the statistics and close any open cv2 device
    Inputs:
        camera (Camera) the Camera being used
        video_out (VideoSaver) the VideoSaver being used
        statistics_engine (StatisticsEngine) the StatisticsEngine being used"""
    if "cv2" in camera.camera_type and camera.video_source.isOpened():
        camera.video_source.release()

    if (video_out is not None and
            video_out.h_video_out is not None and
            video_out.h_video_out.isOpened()):
        video_out.h_video_out.release()
    stat_string = statistics_engine.process_statistics()
    rospy.logerr(stat_string)
    cv2.destroyAllWindows()


def str2bool(s, return_string=False):
    """
    This function is not too correct if the argument is not any of True/False case
    """
    s = str(s)
    if s.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif s.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    elif s is None:
        return None
    elif return_string:
        return s
    else:
        return None