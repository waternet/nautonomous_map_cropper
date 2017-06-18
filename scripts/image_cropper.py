
import math
import rospy

from PIL import Image
import yaml

from passive_map_cropper_node import project_path
from geometry_msgs.msg import Pose2D

from passive_map_cropper_node import publish_initial_position

original_image = None

# Load the entire image of amsterdam so we can use it to crop it.
def load_full_image():
    global original_image
    full_image_string = project_path + "/images/amsterdam_total.png"
    original_image = Image.open(full_image_string)

# def crop_map_edge(edge):
# 	print "crop map edge: " + str(edge)

# 	traffic_id = getTrafficId(edge)

# 	map_data = open_config_file()

# 	first_node = getNode(edge[0])
# 	second_node = getNode(edge[1])

# 	resolution, left_position, right_position, bottom_position, top_position = extract_cropped_image_position(map_data, first_node, second_node)
# 	save_cropped_image(traffic_id, resolution, left_position, right_position, bottom_position, top_position)
# 	save_config_file(traffic_id, map_data, left_position, right_position, bottom_position, top_position)

# def crop_maps():
# 	i = 0
# 	for edge in G.edges():
# 		print str((i+1)) + "/" + str(len(G.edges())) + ": " + str(edge)
# 		i = i + 1
# 		crop_map_edge(edge)

# def crop_map(first_node, second_node):
# 	print "crop map: " + str(first_node) + " " + str(second_node)
# 	traffic_id = getTrafficId(edge)

# 	map_data = open_config_file()

# 	resolution, left_position, right_position, bottom_position, top_position = extract_cropped_image_position(map_data, first_node, second_node)
# 	save_cropped_image(traffic_id, resolution, left_position, right_position, bottom_position, top_position)
# 	save_config_file(traffic_id, map_data, left_position, right_position, bottom_position, top_position)

# Crop the map using a list of points
def crop_map_points(points,	name_map):
    global original_image

    map_data = open_config_file()
    # extract the position of the cropped map
    resolution, left_position, right_position, bottom_position, top_position = extract_cropped_image_positions(map_data, points)
    # save the cropper image based on the positions
    save_cropped_image(original_image, name_map, resolution, left_position, right_position, bottom_position, top_position)
    # save the config file based on the positions
    image_file_name, config_file_name =  save_config_file(name_map, map_data, left_position, right_position, bottom_position, top_position)

    return image_file_name, config_file_name

# Open the config file from the original image
def open_config_file():
	with open(project_path + "/config/amsterdam_total.yaml") as f:
		map_data = yaml.safe_load(f)
	return map_data

# Extract the cropped image positions based on the positions of the nodes in the map.
def extract_cropped_image_positions(map_data, nodes):
    # get map parameters
	resolution = map_data["resolution"]
	map_left = map_data["map_left"]
	map_top = map_data["map_top"]
	map_bottom = map_data["map_bottom"]

    # Get the transformed nodes in the full image frame
	nodes_x = []
	nodes_y = []
	for node in nodes:
		nodes_x.append(node.x - map_left)
		nodes_y.append(map_top - node.y)

	publish_initial_position(nodes[0].x, nodes[0].y, 0);
	
    #TODO add the theta parameter to more efficiently crop the image.
	# edge_width = abs(first_node_x - second_node_x)
	# edge_height = abs(first_node_y - second_node_y)
	# theta = math.atan2( edge_height, edge_width)
	# print "Theta: " + str(theta)

    #TODO dynamic margin
	# margin for top, left, bottom and right
	margin = 50 # magic 50 meters boundary

	# find rectangle that fits the path
	print "Nodes: " + str(min(nodes_x)) + " " + str(max(nodes_x)) + " " + str(max(nodes_y)) + " " + str(min(nodes_y))
	left_position = min(nodes_x) - margin
	right_position = max(nodes_x) + margin
	bottom_position = max(nodes_y) + margin
	top_position = min(nodes_y) - margin

	return resolution, left_position, right_position, bottom_position, top_position 

# def extract_cropped_image_position(map_data, first_node, second_node):

# 	resolution = map_data["resolution"]
# 	map_left = map_data["map_left"]
# 	map_right = map_data["map_right"]
# 	map_bottom = map_data["map_bottom"]
# 	map_top = map_data["map_top"]

# 	map_width = map_right - map_left
# 	map_height = map_top - map_bottom

# 	first_node_x = (first_node.easting - map_left)
# 	first_node_y = (map_top - first_node.northing)
# 	#print "First: " + str(first_node_x) + " " + str(first_node_y)

# 	second_node_x = (second_node.easting - map_left)
# 	second_node_y = (map_top - second_node.northing)
# 	#print "Second: " + str(second_node_x) + " " + str(second_node_y)

# 	edge_width = abs(first_node_x - second_node_x)
# 	edge_height = abs(first_node_y - second_node_y)
# 	theta = math.atan2( edge_height, edge_width)
# 	#print "Theta: " + str(theta)
# 	# margin for top, left, bottom and right
# 	margin = 50 # magic 50 meters boundary

# 	# find rectangle that fits the path
# 	left_position = min(first_node_x,second_node_x) - margin
# 	right_position = max(first_node_x,second_node_x) + margin
# 	bottom_position = max(first_node_y, second_node_y) + margin
# 	top_position = min(first_node_y, second_node_y) - margin

# 	#print "Pixels: x: " + str(left_pixel) + " y: " + str(right_pixel) + " b: " + str(bottom_pixel) + " t: " + str(top_pixel)  

# 	return resolution, left_position, right_position, bottom_position, top_position 

# Save the cropped image based on the cropped positions
def save_cropped_image(original_image, file_name, resolution, left_position, right_position, bottom_position, top_position):
    # create cropped image
	cropped_example = original_image.crop(
		(
			int(left_position / resolution),  
			int(top_position / resolution), 
			int(right_position / resolution), 
			int(bottom_position / resolution)
		)
	)
	cropped_example.save(project_path + "/images/amsterdam_cropped_" + str(file_name) + ".png")

# Save config file based on the cropped positions
def save_config_file(file_name, map_data, left_position, right_position, bottom_position, top_position):
	# create cropped config file
	map_left = map_data["map_left"]
	map_right = map_data["map_right"]
	map_bottom = map_data["map_bottom"]
	map_top = map_data["map_top"]

	map_data["map_left"] = left_position
	map_data["map_right"] = right_position
	map_data["map_bottom"] = bottom_position
	map_data["map_top"] = top_position
	map_data["image"] = "/images/amsterdam_cropped_" + str(file_name) + ".png"
	map_data["origin"] = [left_position+map_left, (map_top - map_bottom) - bottom_position + map_bottom, 0.0]

    # Save the image and config name
	image_file_name = project_path + map_data["image"]
	config_file_name = project_path + "/config/amsterdam_cropped_" + str(file_name) + ".yaml"
 
    # Open the config file and dump the data
	with open(config_file_name, "w") as f:
		yaml.dump(map_data, f)

    # return the image and config file name
	return image_file_name, config_file_name
