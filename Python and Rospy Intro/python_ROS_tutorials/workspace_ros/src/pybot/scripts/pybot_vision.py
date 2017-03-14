#!/usr/bin/python
#
# The MIT License (MIT)
# 
# Copyright (c) 2015 Australian Centre for Robotic Vision
# 

"""
pybot_vision: basic image receiver demo

This demo subscribes to an image topic and display its content in an
OpenCV window.

------------
Mode of use
------------
$ rosrun pybot pybot_vision _topic:=<image_topic> _compressed:=<bool>

_topic: image topic path, default 'camera/rgb/image_color/compressed'
_compressed: tell if topic is a compressed image topic, default true
"""

__author__      = 'Juan David Adarve'
__email__       = 'juan.adarve@anu.edu.au'
__license__     = 'MIT'
__copyright__   = 'Copyright 2015, Australian Centre for Robotic Vision'

import rospy
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class ImageReceiver(object):
    """
    Image receiver class
    
    Objects of this class subscribe to a ROS image topic, either raw or compresssed,
    and call image listeners waiting for new image data to process.
    """
    
    def __init__(self, imageTopic, compressed=False):
        """
        Creates an image receiver subscribing to a specific image topic
        
        ----------------
        PARAMETERS
        ----------------
        imageTopic : string with the image topic to subscribe
        compressed : boolean to specify if the image topic is compressed (default False)
        """
        
        self.__compressedImage = compressed
        self.__imgListeners = list()
        
        if self.__compressedImage:
            self.__subscriber = rospy.Subscriber(imageTopic, CompressedImage, self.callback, queue_size=1)
        else:
            self.__bridge = CvBridge()
            self.__subscriber = rospy.Subscriber(imageTopic, Image, self.callback, queue_size=1)
        
        print('ImageReceiver: subscribed to topic {0}\tcompresssed: {1}'.format(imageTopic, compressed))
    
        
    def callback(self, imgObj):
        """
        Image callback
        
        Each time a new image arrives, all image listeners are called with the
        new image as parameter.
        """
        
        if self.__compressedImage:
            arrData = np.fromstring(imgObj.data, np.uint8)
            #self.image = cv2.imdecode(arrData, cv2.CV_LOAD_IMAGE_COLOR)
            self.image = cv2.imdecode(arrData, cv2.IMREAD_COLOR)
        else:
            try:
                self.image = self.__bridge.imgmsg_to_cv2(imgObj, 'passthrough')
            except CvBridgeError as e:
                print('ImageReceiver: error receiving image (uncompressed): {0}'.format(e))
        
        # broadcast the new image to all listeners
        for l in self.__imgListeners:
            try:
                l(self.image)
            except Exception as e:
                print('ImageReceiver: error broadcasting image: {0}'.format(e))


    def addImageListener(self, listener):
        """
        Add a new imageListener
        
        ----------------
        PARAMETERS
        ----------------
        listener : callable object with signature method(img)
        """
        self.__imgListeners.append(listener)



class ImageViewer(object):
    """
    Image viewer class
    
    Objects of this class receive an image through the imageCallback function
    and display it on screen using OpenCV imshow method.
    """
    
    def __init__(self):
        self.title = 'image'
    
    def imageCallback(self, img):
        """
        Image callback called by the imageReceiver
        
        ----------------
        PARAMETERS
        ----------------
        img : new image data
        """
        cv2.imshow(self.title, img)
        cv2.waitKey(10)

            
###########################################################
# ENTRY POINT
###########################################################
if __name__ == '__main__':
    
    print('pybot_vision: start')
    
    # init ros node
    rospy.init_node('pybot_vision')
    
    topic = rospy.get_param('pybot_vision/topic', 'camera/rgb/image_color/compressed')
    compressed = rospy.get_param('pybot_vision/compressed', True)
    
    # creates an image receiver remotely connected to the image_color topic
    imgReceiver = ImageReceiver(topic, compressed=compressed)

    # creates an image viewer and subscribe it to the callback list of imgRecevier
    imgViewer = ImageViewer()
    imgReceiver.addImageListener(imgViewer.imageCallback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('pybot_vision: keyboard interrupt, shutting down')
    
    cv2.destroyAllWindows()
