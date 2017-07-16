#!/usr/bin/env python
from PIL import Image
import rospy

from image_cropper_service import ImageCropperService
from nautonomous_map_msgs.srv import Crop

def main():
	global image_cropper_service

	rospy.init_node('image_cropper_node')

	# Creat the Image Cropper Service
	image_cropper_service = ImageCropperService()

	# Link the execute service function to the cropping service.
	s = rospy.Service('crop', Crop, image_cropper_service.execute_service)

	rospy.spin()

if __name__ == '__main__':
	main()
