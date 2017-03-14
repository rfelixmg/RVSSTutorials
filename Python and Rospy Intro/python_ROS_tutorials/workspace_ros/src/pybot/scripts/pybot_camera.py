#!/usr/bin/python
#
# The MIT License (MIT)
# 
# Copyright (c) 2015 Australian Centre for Robotic Vision
# 

__author__      = 'Juan David Adarve'
__email__       = 'juan.adarve@anu.edu.au'
__license__     = 'MIT'
__copyright__   = 'Copyright 2015, Australian Centre for Robotic Vision'

import threading
import rospy
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class CameraPublisher(object):
    
    def __init__(self, topic='pybot/camera/image_color/compressed', camindex=0, resolution=(640,480)):
        
        # creates a ROS publisher for compressed images
        self.__publisher = rospy.Publisher(topic, CompressedImage)
        
        self.__capture = cv2.VideoCapture(camindex)
        if self.__capture == None:
            rospy.logerr('CameraPublisher: capture device not found')
            quit()
        else:
            rospy.loginfo('CameraPublisher: capture device found')
        
        #self.__capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, resolution[0])
        #self.__capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.__capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.__capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    
        rospy.loginfo('CameraPublisher: starting capture loop')
        self.__imgThread = threading.Thread(target=self.__imageLoop)
        self.__imgThread.start()
    
    
    def __imageLoop(self):
        """
        Image acquisition and processing loop.
        
        This method constantly reads an image from the capture device and
        compresses it and publishes it in the ROS topic
        """
        
        # 10 Hz frame rate
        self.__rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            
            try:
                # reads a new image from the camera
                self.__imgBGR = self.__capture.read()[1]
                
                if self.__imgBGR != None:
                    
                    # creates a compressed image message
                    imgMsg = CompressedImage()
                    imgMsg.header.stamp = rospy.Time.now()
                    imgMsg.format = 'jpeg'
                    imgMsg.data = np.array(cv2.imencode('.jpg', self.__imgBGR)[1]).tostring()
                     
                    # publish the image
                    self.__publisher.publish(imgMsg)
                    
                    cv2.imshow('camera', self.__imgBGR)
                    cv2.waitKey(10)
                    
                else:
                    rospy.logerr('CameraPublisher: error: no image read from camera')
                    
                self.__rate.sleep()
                
            except Exception as e:
                rospy.logerr('CameraPublisher: error reading image frame: {0}'.format(e.errmsg))

###########################################################
# ENTRY POINT
###########################################################
if __name__ == '__main__':
    
    # init ros node
    rospy.init_node('pybot_camera')
    
    rospy.loginfo('pybot_camera: start')
    
    # read node parameters
    topic = rospy.get_param('pybot_camera/topic', 'pybot/camera/image_color/compressed')
    camindex = rospy.get_param('pybot_camera/camera', 0)
    resolution = rospy.get_param('pybot_camera/resolution', [320, 240])
    
    rospy.loginfo('topic name: {0}'.format(topic))
    rospy.loginfo('camera index: {0}'.format(camindex))
    rospy.loginfo('image resolution: {0}'.format(resolution))
    
    webCamPub = CameraPublisher(topic, camindex, resolution)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('pybot_camera: keyboard interrupt, shutting down')
    
    cv2.destroyAllWindows()
