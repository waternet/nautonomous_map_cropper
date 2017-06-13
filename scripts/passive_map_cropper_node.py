#!/usr/bin/env python

from PIL import Image
import rospy
import utm
import os
import yaml
import gps_common
import dynamic_reconfigure.client

from sensor_msgs.msg import NavSatFix
from math import sqrt
from std_msgs.msg import Float32MultiArray, String

import networkx as nx
import json

from vertex import Vertex

import math

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))

G = nx.Graph()
v = []

original_image = 0

def utmFixCallback(data):
	global current_latitude, current_longitude, map_latitude, map_longitude, cropping, fill_initial_coord, initial_lat, initial_lon

	if fill_initial_coord:
		initial_lat = data.latitude
		initial_lon = data.longitude
		fill_initial_coord = False

	current_longitude = data.longitude
	current_latitude = data.latitude

	utm_map_position = utm.from_latlon(map_latitude, map_longitude)
	utm_current_position = utm.from_latlon(current_latitude, current_longitude)

	x_diff = utm_map_position[0] - utm_current_position[0]
	y_diff = utm_map_position[1] - utm_current_position[1]
	abs_diff = sqrt(x_diff * x_diff + y_diff * y_diff)

	if abs_diff > 200 and not cropping and (initial_lat != data.latitude and initial_lon != data.longitude):
		cropping = True
		crop()
		cropping = False



def crop():
	global map_latitude, map_longitude, current_latitude, current_longitude

	print "cropping started"

	with open(project_path + "/config/amsterdam_total.yaml") as f:
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

	map_data["image"] = "amsterdam_cropped.png"
	map_data["origin"] = [((top - bottom) / (2 * margin_multiplier)), ((left - right) / (2 * margin_multiplier)), 0.0]
	map_data["gps_origin"] = [current_latitude, current_longitude]

	with open(project_path + "/config/amsterdam_cropped.yaml", "w") as f:
		yaml.dump(map_data, f)

	with open(os.path.dirname(project_path) + "/nautonomous_configuration/config/map_amsterdam/amsterdam_cropped.yaml", "w") as f:
		yaml.dump(map_data, f)	

	map_latitude = current_latitude
	map_longitude = current_longitude	

	msg = String()
	msg.data = os.path.dirname(project_path) + "/nautonomous_configuration/config/map_amsterdam/amsterdam_cropped.yaml"
	pub_new_map.publish(msg)

	print "cropping finished"

def getTrafficId(edge):
    for node in v:
        if node.id == edge[0]:
            l1 = node.adjacent
        elif node.id == edge[1]:
            l2 = node.adjacent
    return list(set(l1).intersection(l2))[0]

def getNode(searchNode):
	for node in v:
		if node.id == searchNode:
			return node

#     for item in traffic["data"]:
#         if(item["trafficlink_id"] == traffic_id[0]):
#             return euclideanDistance(first, second) * (item["traffic"] / 4 + 1)
            
#     return 0;

def crop_edge_map(edge):
	global original_image

	traffic_id = getTrafficId(edge)

	# get full map 
	with open(project_path + "/config/amsterdam_total.yaml") as f:
		map_data = yaml.safe_load(f)

	resolution = map_data["resolution"]
	map_left = map_data["map_left"]
	map_right = map_data["map_right"]
	map_bottom = map_data["map_bottom"]
	map_top = map_data["map_top"]
	#print str(map_left) + " " + str(map_right) + " " + str(map_bottom) + " " + str(map_top)

	map_width = map_right - map_left
	map_height = map_top - map_bottom
	#print "Map width: " + str(map_width)
	#print "Map height: " + str(map_height)
	
	first_node = getNode(edge[0])
	second_node = getNode(edge[1])
	#print "first node " + str(first_node.easting) + " " + str(first_node.northing) 
	#print "second node " + str(second_node.easting) + " " + str(second_node.northing) 

	first_node_x = (first_node.easting - map_left) / resolution
	first_node_y = (map_top - first_node.northing) / resolution
	#print "First: " + str(first_node_x) + " " + str(first_node_y)

	second_node_x = (second_node.easting - map_left) / resolution
	second_node_y = (map_top - second_node.northing) / resolution
	#print "Second: " + str(second_node_x) + " " + str(second_node_y)

	# margin for top, left, bottom and right
	margin = 50 # magic 50 meters boundary
	margin_pixels = margin / resolution 

	# find rectangle that fits the path
	left_pixel = min(first_node_x,second_node_x) - margin_pixels
	right_pixel = (max(first_node_x,second_node_x) + margin_pixels)
	bottom_pixel = max(first_node_y, second_node_y) + margin_pixels
	top_pixel = min(first_node_y, second_node_y) - margin_pixels

	#print "Pixels: x: " + str(left_pixel) + " y: " + str(right_pixel) + " b: " + str(bottom_pixel) + " t: " + str(top_pixel)  

	# create cropped image
	cropped_example = original_image.crop(
		(
			int(left_pixel),  
			int(top_pixel), 
			int(right_pixel), 
			int(bottom_pixel)
		)
	)
	cropped_example.save(project_path + "/images/amsterdam_cropped_" + str(traffic_id) + ".png")
	
	# create cropped config file
	map_data["map_left"] = min(first_node.easting, second_node.easting) - margin
	map_data["map_right"] = max(first_node.easting, second_node.easting) + margin
	map_data["map_bottom"] = min(first_node.northing, second_node.northing) - margin
	map_data["map_top"] = max(first_node.northing, second_node.northing) + margin

	map_data["image"] = "/images/amsterdam_cropped_" + str(traffic_id) + ".png"
	map_data["origin"] = [map_data["map_left"], map_data["map_bottom"], 0.0]

	with open(project_path + "/config/amsterdam_cropped_" + str(traffic_id) + ".yaml", "w") as f:
		yaml.dump(map_data, f)

def crop_maps():
	global original_image
	i = 0
	full_image_string = project_path + "/images/amsterdam_total.png"
	original_image = Image.open(full_image_string)
	print "loaded map"
	for edge in G.edges():
		print str((i+1)) + "/" + str(len(G.edges())) + ": " + str(edge)
		i = i + 1
		crop_edge_map(edge)

def convertGPSStringToUTM(gpsString):
	gpsPosition = map(float, gpsString.split(",", 1))
	utmPosition = utm.from_latlon(gpsPosition[0], gpsPosition[1])[:2] # only use the first two elements of the tuple: easting and northing (in that order)
	utmString = str(utmPosition[0]) + "," + str(utmPosition[1])
	return utmString, utmPosition[0], utmPosition[1]

def convertGPSArrayToUTM(gpsArray):
	result = []
	for gpsString in gpsArray:
		result.append(convertGPSStringToUTM(gpsString)[0])
	return result

def euclideanDistance(start, end):
    startString = start.split(",")
    endString = end.split(",")
    return math.sqrt(math.pow((float(startString[0])-float(endString[0])), 2)+math.pow((float(startString[1])-float(endString[1])),2))
    
def construct_graph():
	global v, G
	print "Constructing Graph"
	with open(project_path + '/data/intersections.json') as data_file:    
		intersection_data = json.load(data_file)
		for node in intersection_data:
			node_id_string, node_id_easting, node_id_northing = convertGPSStringToUTM(node["id"])
			nodes_connected = convertGPSArrayToUTM(node["connected"])
			v.append(Vertex(node_id_string, node_id_easting, node_id_northing, node["adjacent"], nodes_connected))
			G.add_node(node_id_string, pos = (node_id_easting, node_id_northing))

		for node in v:
			for connected in node.connected:
				G.add_edge(node.id, connected, weight = (euclideanDistance(node.id,connected)))

def main():

	rospy.init_node('passive_map_cropper')
	
	construct_graph()

	crop_maps()

	print "Done, cropped all " + str(len(G.edges())) + " nodes."

if __name__ == '__main__':
	main()
