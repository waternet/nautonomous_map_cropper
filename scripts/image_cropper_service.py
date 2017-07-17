import os

import rospy
import rospkg

rospack = rospkg.RosPack()
nautonomous_configuration_path = rospack.get_path('nautonomous_configuration')

from PIL import Image
import yaml

from nautonomous_map_msgs.srv import CropResponse

class ImageCropperService:

	# constants
	image_file_extension_ = ".png"
	config_file_extension_ = ".yaml"

	map_folder_ = "/config/navigation/map/"
	file_name_adjustment_ = "_cropped_"

	# map
	map_image_ = None
	map_data_ = None

	def __init__(self):
		self.load_params()
		self.map_image_ = self.load_map_image()

	def load_params(self):
		self.map_name_ = rospy.get_param('~map_image_name_param', 'amsterdam')
		self.negate_image_ = rospy.get_param('~negate_image_param', False)

	# Load the entire image of amsterdam so we can use it to crop it.
	def load_map_image(self):
		
		full_image_string = nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.image_file_extension_
		print full_image_string
		return Image.open(full_image_string)

	# Create the service so other nodes can request to crop the map.
	def execute_service(self, request):

		route = request.route
		name_map = request.name

		config_name = nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.config_file_extension_
		if(os.path.isfile(config_name)):
			print "Config already exists, returning existing configuration file."
			return CropResponse(config_name)
	
		map_data_ = self.open_map_config_file()

		# extract the position of the cropped map
		resolution, left_position, right_position, bottom_position, top_position = self.extract_cropped_image_positions(map_data_, route)

		# save the cropper image based on the positions
		self.save_cropped_image(name_map, resolution, left_position, right_position, bottom_position, top_position)

		# save the config file based on the positions
		config_file_name =  self.save_config_file(name_map, map_data_, left_position, right_position, bottom_position, top_position)

		return CropResponse(config_file_name)

	# Open the config file from the map image
	def open_map_config_file(self):
		with open(nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.config_file_extension_) as f:
			map_data_ = yaml.safe_load(f)
		return map_data_

	# Extract the cropped image positions based on the positions of the nodes in the map.
	def extract_cropped_image_positions(self, map_data_, route):
		# get map parameters
		resolution = map_data_["resolution"]
		map_left = map_data_["map_left"]
		map_top = map_data_["map_top"]
		map_bottom = map_data_["map_bottom"]

		# Get the transformed nodes in the full image frame
		nodes_x = []
		nodes_y = []
		for node in route:
			nodes_x.append(node.x - map_left)
			nodes_y.append(map_top - node.y)
		
		#TODO add the theta parameter to more efficiently crop the image.
		# edge_width = abs(first_node_x - second_node_x)
		# edge_height = abs(first_node_y - second_node_y)
		# theta = math.atan2( edge_height, edge_width)
		# print "Theta: " + str(theta)

		#TODO dynamic margin
		# margin for top, left, bottom and right
		margin = 50 # magic 50 meters boundary

		# find rectangle that fits the path
		left_position = min(nodes_x) - margin
		right_position = max(nodes_x) + margin
		bottom_position = max(nodes_y) + margin
		top_position = min(nodes_y) - margin

		return resolution, left_position, right_position, bottom_position, top_position

	# Save the cropped image based on the cropped positions
	def save_cropped_image(self, name_map, resolution, left_position, right_position, bottom_position, top_position):
		# create cropped image
		cropped_example = self.map_image_.crop(
			(
				int(left_position / resolution),  
				int(top_position / resolution), 
				int(right_position / resolution), 
				int(bottom_position / resolution)
			)
		)
		cropped_example.save(nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.image_file_extension_)

	# Save config file based on the cropped positions
	def save_config_file(self, name_map, map_data_, left_position, right_position, bottom_position, top_position):
		# create cropped config file
		map_left = map_data_["map_left"]
		map_right = map_data_["map_right"]
		map_bottom = map_data_["map_bottom"]
		map_top = map_data_["map_top"]

		map_data_["map_left"] = left_position
		map_data_["map_right"] = right_position
		map_data_["map_bottom"] = bottom_position
		map_data_["map_top"] = top_position
		map_data_["image"] = nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.image_file_extension_
		map_data_["origin"] = [left_position + map_left, map_top - bottom_position, 0.0]
		
		negate_image_value = 0

		if self.negate_image_:
			negate_image_value = 1

		map_data_["negate"] = negate_image_value

		# Save the image and config name
		config_name = nautonomous_configuration_path + self.map_folder_ + self.map_name_ + self.file_name_adjustment_ + name_map + self.config_file_extension_
	
		# Open the config file and dump the data
		with open(config_name, "w") as f:
			yaml.dump(map_data_, f)

		# return the image and config file name
		return config_name
