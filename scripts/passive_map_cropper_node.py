#!/usr/bin/env python

from PIL import Image
import rospy

from geometry_msgs.msg import Pose2D
from nautonomous_map_msgs.srv import CropMapPoints

import image_cropper

initial_pose_pub = None

def publish_initial_position(x, y, theta):
	global initial_pose_pub
	print "Publish initial position: " + str(x) + " " + str(y) + " " + str(theta)
	initial_pose_pub.publish(Pose2D(x, y, theta))		

# Create the service so other nodes can request to crop the map.
def crop_map_points_service(request):
	image_file_name, config_file_name, x, y, theta = image_cropper.crop_map_points(request.pathLocations, request.operation_name)
	publish_initial_position(x, y, theta)
	return CropMapPointsResponse(image_file_name, config_file_name)
	
def main():
	global initial_pose_pub
	rospy.init_node('nautonomous_map_cropper')

	print "Passive map cropper"
	image_cropper.load_full_image()

	initial_pose_pub = rospy.Publisher('/map/cropper/initial_position', Pose2D, queue_size=10)

	s = rospy.Service('/map/cropper/crop_map_points', CropMapPoints, crop_map_points_service)

	rospy.spin()

if __name__ == '__main__':
	main()
