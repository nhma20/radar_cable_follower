#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import copy
import matplotlib.pyplot as plt
import sys
import os
import math 
import matplotlib.lines as mlines

global logging_dir
logging_dir = str(sys.argv[1])
global image_width
image_width  = 1920
global image_height
image_height  = 1080
global img_dims
img_dims = (image_width, image_height)


global img_hfov 
img_hfov = 1.3962634016

global lowest_time
lowest_time = 9999999999999
for filename in os.listdir(logging_dir):
	if ".jpg" in filename:
		if int(filename.replace(".jpg", "")) < lowest_time:
			lowest_time = int(filename.replace(".jpg", ""))


def project_points(data_file):

	timestamps = []
	point = []
	point_array = [] 
	all_points = [] # point_array[3][2][1] ; third timestamped point set, point index 2, y value
	line_counter = 0

	with open(os.path.join(logging_dir + data_file), encoding='utf8') as f:
		for line in f:

			linestring = line.strip()

			if "t" in linestring:
				continue

			linestring = line.strip().split(",")


			if int(linestring[0]) < lowest_time:
				continue

			timestamps.append(linestring[0])

			number_points = int(linestring[1])

			for i in range(number_points):
				
				point.append( float(linestring[2+i*3+0]) )
				point.append( float(linestring[2+i*3+1]) )
				point.append( float(linestring[2+i*3+2]) )

				point_array.append(point)

				point = []

			all_points.append(point_array)

			point_array = []

			line_counter = line_counter + 1

	#print(all_points[3][2][1]) ## fits


	h_focal_length = (img_dims[0] * 0.5) / math.tan(img_hfov * 0.5 ); # in pixels

	objects_dists = []
	objects_xz_angle = []
	objects_yz_angle = []

	for i in range(len(all_points)):

		for j in range(len(all_points[i])):

			current_dist = math.sqrt( pow(all_points[i][j][0], 2) + pow(all_points[i][j][1], 2) + pow(all_points[i][j][2], 2) )

			objects_dists.append(current_dist)
			objects_xz_angle.append( math.asin( all_points[i][j][0] / math.sqrt(pow(all_points[i][j][0],2) + pow(all_points[i][j][2],2))) )
			objects_yz_angle.append( math.asin(all_points[i][j][1] / math.sqrt(pow(all_points[i][j][1],2) + pow(all_points[i][j][2],2))) )

			# if( current_dist < closest_dist ){
			# 	closest_dist = current_dist;
			# 	this->closest_idx = i;
			# }


	x_px_vec = []
	y_px_vec = []

	for i in range(len(objects_dists)):

		# horizontal pixel
		x_depth = math.sin(objects_xz_angle[i]) * objects_dists[i]
		y_depth = math.cos(objects_xz_angle[i]) * objects_dists[i]

		xy_ratio = 999999999999

		if y_depth != 0:
			xy_ratio = x_depth/y_depth
		
		x_px_vec.append( -1 * xy_ratio * h_focal_length  + image_width/2); # -1 to mirror (pinhole stuff)
		
		# vertical pixel
		x_depth = math.sin(objects_yz_angle[i]) * objects_dists[i] 
		y_depth = math.cos(objects_yz_angle[i]) * objects_dists[i]

		xy_ratio = 999999999999

		if y_depth != 0:
			xy_ratio = x_depth/y_depth
		
		y_px_vec.append( -1 * xy_ratio * h_focal_length + image_height/2); # -1 to mirror (pinhole stuff)


	x_px = image_width/2 - np.asarray(y_px_vec) + image_height/2
	y_px = image_height/2 - np.asarray(x_px_vec) + image_width/2

	return x_px, y_px



def get_direction(data_file):

	with open(os.path.join(logging_dir + data_file), encoding='utf8') as f:
		for line in f:

			linestring = line.strip()

			if "t" in linestring:
				continue

			linestring = line.strip().split(",")

			if int(linestring[0]) < lowest_time:
				continue

			return linestring[1]
			



print("transformed")
x_px_trans, y_px_trans = project_points("transformed_points.txt")
print("projected")
x_px_proj, y_px_proj = project_points("projected_points.txt")
print("estimated")
x_px_est, y_px_est = project_points("points_est.txt")

print("direction")
yaw = float(get_direction("cable_yaw.txt"))
print(yaw)
p1 = [image_width/2, image_height/2]
p2 = [0, 0]
p2[0] = (math.sin(yaw) * 10) + image_width/2
p2[1] = (math.cos(yaw) * 10) + image_height/2



img = cv.imread(logging_dir + "/" + str(lowest_time) + ".jpg")
#img = cv.resize(img,(640,480),cv.INTER_AREA)
img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
#img = plt.imread(logging_dir + "/" + str(lowest_time) + ".jpg")
fig, ax = plt.subplots()
ax.imshow(img, extent=[0, image_width, 0, image_width])
ax.scatter(x_px_trans, y_px_trans, linewidth=0.000001, color='red', label='raw points')
ax.scatter(x_px_proj, y_px_proj, linewidth=0.000001, color='yellow', label='projected points')
ax.scatter(x_px_est, y_px_est, linewidth=0.000001, color='green', label='estimated points')


xmin, xmax = ax.get_xbound()

if(p2[0] == p1[0]):
	xmin = xmax = p1[0]
	ymin, ymax = ax.get_ybound()
else:
	ymax = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmax-p1[0])
	ymin = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmin-p1[0])

l = mlines.Line2D([xmin,xmax], [ymin,ymax], color='lawngreen', label='Cable yaw')
ax.add_line(l)

ax.legend()

ax.imshow(img)

plt.show()
