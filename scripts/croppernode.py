#!/usr/bin/env python

from PIL import Image
import rospy
import utm
import os
import yaml
import gps_common

from sensor_msgs.msg import NavSatFix
from math import sqrt
from std_msgs.msg import Float32MultiArray

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))

pub = rospy.Publisher('cropper_map_gps', Float32MultiArray, queue_size=10)

map_latitude = 52.364998
map_longitude = 4.879421

current_latitude = 52.366734
current_longitude = 4.877543


def gpsFixCallback(data):
	global current_latitude, current_longitude, map_latitude, map_longitude
	current_longitude = data.longitude
	current_latitude = data.latitude

	utm_map_position = utm.from_latlon(map_latitude, map_longitude)
	utm_current_position = utm.from_latlon(current_latitude, current_longitude)

	x_diff = utm_map_position[0] - utm_current_position[0]
	y_diff = utm_map_position[1] - utm_current_position[1]
	abs_diff = sqrt(x_diff * x_diff + y_diff * y_diff)
	#print "Diff abs: ", int(abs_diff)
	if abs_diff > 200:
		crop()



def crop():
	with open(project_path + "/config/total.yaml") as f:
		map_data = yaml.safe_load(f)

	utm_val = utm.from_latlon(current_latitude, current_longitude)	

	resolution = map_data["resolution"]
	origin = map_data["origin"]

	print "Path: ", os.path.dirname(project_path)

	#check if inside map
	if utm_val[0] < map_data["map_left"] and utm_val[0] > map_data["map_right"] and utm_val[1] > map_data["map_top"] and utm_val[1] < map_data["map_bottom"]:
		exit # possible?

	test_image = project_path + "/images/amsterdam_total.png"
	original = Image.open(test_image)

	#Crop margin in meters around center 
	margin = 500;
	margin_multiplier = (1/resolution)

	image_height_total = (map_data["map_top"] - map_data["map_bottom"]) * margin_multiplier
	image_width_total = (map_data["map_right"] - map_data["map_left"]) * margin_multiplier

	print "Image size: ", int(image_width_total), "/", int(image_height_total)

	#location on image in pix
	loc_pix_x = (map_data["map_left"] - utm_val[0]) * -margin_multiplier
	loc_pix_y = (map_data["map_top"] - utm_val[1]) * margin_multiplier

	print "Loc om image: ", int(loc_pix_x), "/", int(loc_pix_y)

	left = loc_pix_x - margin * margin_multiplier
	right = loc_pix_x + margin * margin_multiplier
	top = loc_pix_y - margin * margin_multiplier
	bottom = loc_pix_y + margin * margin_multiplier

	#Don't crop outside image
	if left < 0:
		left = 0
	if right > image_width_total:
		right = image_width_total
	if top < 0:
		top = 0
	if bottom > image_height_total:
		bottom = image_height_total

	print "INFO left: ", int(left), "   right:  ", int(right), "   top: ", int(top), "   bottom: ", int(bottom)
	
	cropped_example = original.crop((int(left), int(top), int(right), int(bottom)))
	cropped_example.save(project_path +"/images/amsterdam_cropped.png")
	#Also safe in config package
	cropped_example.save(os.path.dirname(project_path) + "/nautonomous_configuration/config/map_amsterdam/amsterdam_cropped.png")

	print "cropped";

	map_data["image"] = "amsterdam_cropped.png"
	map_data["origin"] = [((top - bottom) / (2 * margin_multiplier)), ((left - right) / (2 * margin_multiplier)), 0.0]
	map_data["gps_origin"] = [current_latitude, current_longitude]

	with open(project_path + "/config/amsterdam_cropped.yaml", "w") as f:
		yaml.dump(map_data, f)

	with open(os.path.dirname(project_path) + "/nautonomous_configuration/config/map_amsterdam/amsterdam_cropped.yaml", "w") as f:
		yaml.dump(map_data, f)	

	map_latitude = current_latitude
	map_longitude = current_longitude	

	map_center_values = Float32MultiArray()
	map_center_values.data.insert(0, map_latitude)
	map_center_values.data.insert(1, map_longitude)

	pub.publish(map_center_values)

	#restart map server(?)
	os.system("rosrun map_server map_server " + os.path.dirname(project_path) + "/nautonomous_configuration/config/map_amsterdam/amsterdam_cropped.yaml&")


def main():
	global current_latitude, current_longitude, map_latitude, map_longitude

	rospy.init_node('map_cropper')
	sub = rospy.Subscriber("gps/fix", NavSatFix, gpsFixCallback)
	os.system("rosrun map_server map_server " + os.path.dirname(project_path) + "/nautonomous_configuration/config/map_amsterdam/amsterdam_cropped.yaml&")

	location = [52.404601, 4.863270]

	current_latitude = location[0]
	current_longitude = location[1]
	map_latitude = location[0]
	map_longitude = location[1]
	crop()

	rospy.spin()

if __name__ == '__main__':
	main()
