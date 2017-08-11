#!/usr/bin/env python

import sys
# Prevent pyc files being generated!
sys.dont_write_bytecode = True

from PIL import Image
import rospy

from image_cropper_service import ImageCropperService
from nautonomous_map_msgs.srv import Crop

from geometry_msgs.msg import Pose2D

## debug
class DebugRequest():
	#route = [Pose2D(628329,5803523,0), Pose2D(628649, 5803234,0), Pose2D(629094, 5803149,0), Pose2D(629497,5803283,0)]
	route = [Pose2D(628329, 5803523, 0), Pose2D(628649, 5803232, 0), Pose2D(628649, 5803232, 0), Pose2D(629095, 5803152, 0)]
	
	name = "debug_rotated"

if __name__ == '__main__':
	rospy.init_node('image_cropper_node')

	# Create the Image Cropper Service
	image_cropper_service = ImageCropperService()

	# Link the execute service function to the cropping service.
	s = rospy.Service('crop', Crop, image_cropper_service.execute_service)
	
	debug_request = DebugRequest()
	image_cropper_service.execute_service(debug_request)

	print "cropped"

	rospy.spin()
