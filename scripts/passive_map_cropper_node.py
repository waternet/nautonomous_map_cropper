#!/usr/bin/env python

from PIL import Image
import rospy
import os

from nautonomous_map_cropper.srv import *
from geometry_msgs.msg import Pose2D

initial_pose_pub = None

import image_cropper

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))

def publish_initial_position(x,y, theta):
	global initial_pose_pub
	initial_pose_pub = rospy.Publisher('nautonomous_propulsion_sim/initial_position', Pose2D, queue_size=10)

	print "Publish initial position: " + str(x) + " " + str(y) + " " + str(theta)
	rospy.sleep(1)
	initial_pose_pub.publish(Pose2D(x, y, theta))		

# Create the service so other nodes can request to crop the map.
def crop_map_points_service(request):
	image_file_name, config_file_name = image_cropper.crop_map_points(request.pathLocations, request.operation_name)
	return CropMapPointsResponse(image_file_name, config_file_name)
	
def main():
	global initial_pose_pub
	rospy.init_node('passive_map_cropper')
	print "Passive map cropper"
	image_cropper.load_full_image()
	

	s = rospy.Service('crop_map_points', CropMapPoints, crop_map_points_service)

	rospy.spin()

if __name__ == '__main__':
	main()
