#!/usr/bin/env python

from PIL import Image
import rospy

from geometry_msgs.msg import Pose2D
from nautonomous_map_msgs.srv import Crop, CropResponse

import image_cropper	

# Create the service so other nodes can request to crop the map.
def crop_map_points_service(request):
	config_name = image_cropper.crop_map_points(request.route, request.name)

  	return CropResponse(config_name)
	
def main():
	global initial_pose_pub
	rospy.init_node('passive_map_cropper_node')

	original_image_name = rospy.get_param('~original_image_name', 50)
	test = rospy.get_param('~test_map_param', 50)


	print "Passive map cropper"
	image_cropper.load_original_image(original_image_name, test)

	s = rospy.Service('crop_service', Crop, crop_map_points_service)

	rospy.spin()

if __name__ == '__main__':
	main()
