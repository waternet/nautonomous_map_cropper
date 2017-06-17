#!/usr/bin/env python

from PIL import Image
import rospy
import os

from nautonomous_map_cropper.srv import *

import image_cropper

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))

# Create the service so other nodes can request to crop the map.
def crop_map_points_service(request):
	image_file_name, config_file_name = image_cropper.crop_map_points(request.pathLocations, request.operation_name)
	return CropMapPointsResponse(image_file_name, config_file_name)
	
def main():

	rospy.init_node('passive_map_cropper')
	
	image_cropper.load_full_image()

	s = rospy.Service('crop_map_points', CropMapPoints, crop_map_points_service)

	rospy.spin()

if __name__ == '__main__':
	main()
