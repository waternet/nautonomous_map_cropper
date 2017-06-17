
import utm

import math

import networkx as nx
import json

from vertex import Vertex

G = nx.Graph()
v = []

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
