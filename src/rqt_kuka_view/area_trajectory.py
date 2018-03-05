# input parameter is of type list in the form [[x1, y1], [x2, y2], ... , [xn, yn]]
# where each [xk, yk] point represents a point along the parameter of the 
# area that encloses the desired path trajectory
# returns the desired trajectory to move along the entire area
# None is appended on the return list each time the robot needs to change direction

import operator
import numpy as np

def calculate_trajectory(parameter):
	x_parameter = sorted(parameter, key=lambda x: x[0])
	y_parameter = sorted(parameter, key=lambda x: x[1])
	trajectory = []
	x = []
	y = []
	finished = False
	switch = False
	X = False
	boundary_offset = 10 # do not increase
	line_offset = 20 # change to increase/decrease gap between lines
	start = 0
	skip = 1
	end = 0
	y_old = 0
	x_old = 0
	trajectory = []
	x = []
	y = []
	# user still needs to choose area, return and prompt
	if len(parameter) == 0:
		return None
	# find minimum side to wipe across
	if (x_parameter[-1][0] - x_parameter[0][0]) < (y_parameter[-1][1] - y_parameter[0][1]):
		# X is minimum side, move from y_min to y_max value on each pass
		X = True
	# 	for item in y_parameter[:-1]:
	# 		if (item[1] - y_parameter[1][1]) < boundary_offset:
	# 			skip += 1
	# 		else:
	# 			end = skip
	# 			break
	# else:
	# 	for item in x_parameter[:-1]:
	# 		if (item[0] - x_parameter[1][0]) < boundary_offset:
	# 			skip += 1
	# 		else:
	# 			end = skip
	# 			break

	while not finished:
		if end < len(y_parameter) or end < len(x_parameter):
			if X:
				for item in y_parameter[start:-skip]:
					if (item[1] - y_parameter[start][1]) < boundary_offset:
						skip += 1
					else:
						break
				end = start + skip
				section = y_parameter[start:end]
			else:
				section = x_parameter[start:end]
				for item in x_parameter[start:-skip]:
					if (item[0] - x_parameter[start][0]) < boundary_offset:
						skip += 1
					else:
						break
				end = start + skip
				section = x_parameter[start:end]
		else: # handle last few coordinates if any remaining
			if X:
				section = y_parameter[start:]
				finished = True
			else:
				section = x_parameter[start:]
				finished = True
		if not section:
			break
		for c in section:
			x.append(c[0])
			y.append(c[1])
		# handle if large gap in between points
		if X:
			y_new = min(y)
			if ((y_new - y_old) > line_offset) and (y_old != 0):
				val = y_old + line_offset
				while val < y_new:
					trajectory, switch = add_coords(trajectory, min(x), max(x), val, val, switch)
					old_val = val
					val += line_offset
			trajectory, switch = add_coords(trajectory, min(x), max(x), y_new, y_new, switch)
			y_old = y_new
		else:
			x_new = min(x)
			if ((x_new - x_old) > line_offset) and (x_old != 0):
				val = x_old +line_offset
				while val < x_new:
					trajectory, switch = add_coords(trajectory, val, val, min(y), max(y), switch)
					old_val = val
					val += line_offset
			trajectory, switch = add_coords(trajectory, x_new, x_new, min(y), max(y), switch)
			x_old = x_new	
		start = end
		skip = 1
		del x[:]
		del y[:]
		del section[:]

	return trajectory

def add_coords(trajectory, x_min, x_max, y_min, y_max, switch):
	# Maybe need to add points inbetween min and max values to aid in path planning?
	if switch:
		trajectory.append((x_min, y_min))
		trajectory.append((x_max, y_max))
		switch = False
	else:
		trajectory.append((x_max, y_max))
		trajectory.append((x_min, y_min))
		switch = True
	trajectory.append(None)
	return trajectory, switch
