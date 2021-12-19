#!/usr/bin/env python3
import rospy
import os

import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import albumentations as albu
import torch
import segmentation_models_pytorch as smp

def to_tensor(x, **kwargs):
    return x.transpose(2, 0, 1).astype('float32')

class ImageSegmenter:

    def __init__(self): 
        os.environ['CUDA_VISIBLE_DEVICES'] = '0'
        # Create subscriber

        # Create publisher

        # Load model

        # Define device variable

        self.bridge = CvBridge()
      
        ENCODER = 'efficientnet-b0'
        ENCODER_WEIGHTS = 'imagenet'
        preprocessing_fn = smp.encoders.get_preprocessing_fn(ENCODER, ENCODER_WEIGHTS)

        test_transform = [
            albu.PadIfNeeded(384, 480),
            albu.Lambda(image=preprocessing_fn),
            albu.Lambda(image=to_tensor),
        ]
        self.augmentation = albu.Compose(test_transform)

        rospy.init_node('smp_test', anonymous=True)


    def image_callback(self, image_message):

        # Conversion from ROS msg to opencv
     
        # Performm all necessary augmentations like padding to match network's input
        sample = self.augmentation(image=cv_image)
        np_image = sample['image']

        # image to torch
        
        # prediction
     
        # getting the mask
        
        # conversion to BGR, uint8
        
        # conversion to ROS msg
        
        # publishing
     
if __name__ == '__main__':
    imageSegmenter = ImageSegmenter()
    rospy.spin()