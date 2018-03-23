#######################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#######################################################################

from __future__ import print_function
import rospy, subprocess, sys, cv2
import numpy as np
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from .area_trajectory import calculate_trajectory

##### LEARN #####
L_IMG_WIDTH = 300
L_CROP_HEIGHT = 180
MAX_IMG = 150
##### DETECT #####
D_IMG_WIDTH = 300
D_CROP_HEIGHT = 180
D_CROP_WIDTH = 300
##### TASK #####
T_IMG_WIDTH = 360
DEFAULT_NAME = "tool"


class KukaViewWidget(QWidget):
	def __init__(self):
		super(KukaViewWidget, self).__init__()
		self.setWindowTitle('KUKA Interface')
		self.main_layout = QVBoxLayout()
		self.mode = 'learn'
		##### LEARN VARIABLES #####
		self.l_video_1 = QLabel()
		self.l_video_2 = QLabel()
		self.l_video_3 = QLabel()
		self.l_user_input = QLineEdit()
		self.l_done_button = QPushButton('DONE')
		self.l_learning_button = QPushButton('Start Learning')
		self.currently_learning = False # start/stop learning an object
		self.num_objects = 0 # count the number of objects learned
		##### DETECT VARIABLES #####
		self.d_video_1 = QLabel()
		self.d_video_2 = QLabel()
		self.d_video_3 = QLabel()
		self.d_stop_button = QPushButton('DONE')
		self.d_detecting_button = QPushButton('Start Detecting')
		self.currently_detecting = False # start/stop detection
		##### TASK VARIABLES #####
		self.t_video_1 = QLabel()
		self.t_send_button = QPushButton('Send')
		self.pen = QPen(Qt.red)
		self.pen.setWidth(8)
		self.pen.setCapStyle(Qt.RoundCap)
		self.pen.setJoinStyle(Qt.RoundJoin)
		self.final_path = []
		self.path = []
		self.draw = False
		self.draw_all = False
		self.send_path = False
		self.calculate_area = False
		self.offset = [0,0]
		self.object_id = None
		self.bbox_ready = False
		self.obj_layout = None
		###### NEEDED????  #######
		self.num = 4
		self.cls = ['cookie','candy','chocolate','mountain'] # list of object names
		##### SHARED VARIABLES #####
		self.bridge = CvBridge()
		self.initialize_widgets()
		self.setLayout(self.main_layout)
		######################## setup publishers and subscribers ########################
		##### LEARN #####
		self.pub_learn = rospy.Publisher('/command_learn', String, queue_size=10)
		self.sub_learn1 = rospy.Subscriber('/cam1/camera/image_raw/compressed',  CompressedImage, self.cb_learn1, queue_size = 1)
		self.sub_learn2 = rospy.Subscriber('/crop', Image, self.cb_learn2, queue_size = 1)
		self.sub_learn3 = rospy.Subscriber('/map', Image, self.cb_learn3,  queue_size = 1)
		self.sub_learn4 = rospy.Subscriber('/feature_progress', String, self.cb_progress1, queue_size = 1)
		self.sub_learn5 = rospy.Subscriber('/learning_progress', String, self.cb_progress2, queue_size = 1)
		##### DETECT #####
		self.pub_detect = rospy.Publisher('/command_det', String, queue_size=10)
		self.sub_detect1 = rospy.Subscriber('/kinect2/hd/image_color_rect/compressed', CompressedImage, self.cb_detect1, queue_size = 1)
		self.sub_detect2 = rospy.Subscriber('/cl_image', Image, self.cb_detect2,  queue_size = 1)
		self.sub_detect3 = rospy.Subscriber('/imgnet_image', Image, self.cb_detect3,  queue_size = 1)
		##### TASK #####
		self.pub_task1 = rospy.Publisher('/command_task', String, queue_size=10)
		self.pub_task2 = rospy.Publisher('/chris/targetPoints_2D', String, queue_size=1000)
		self.sub_task1 = rospy.Subscriber('/camera/rgb/image_rect_color/compressed', CompressedImage, self.cb_task1, queue_size = 1)
		self.sub_task2 = rospy.Subscriber('/bbox', String, self.cb_task2,  queue_size = 10)
		##### COMMAND CENTER #####
		self.pub_init = rospy.Publisher('/command_init', String, queue_size=10)
################################################################
#
# TASK MODE
#
#################### TASK METHODS ####################
	def publish_task_check(self):
		""" check if path is ready to publish """
		if self.send_path:
			self.send_path = False
			tmp = self.final_path[:]
			#print 'before mapping: ', tmp
			mapped_path = self.map_path(tmp)
			path = self.string_converter(mapped_path)
			#print 'mapped: ', path
			print("path:---------")
			print(path)
			self.pub_task2.publish(str(path))
			self.pub_init.publish('grasp:' + self.object_id)

	def map_path(self, vector):
		num = len(vector)
		xrange_input = self.p_size.width()
		xrange_output = self.i_size.width()
		yrange_input = self.p_size.height()
		yrange_output = self.i_size.height()
		mapped = []
		if self.i_size != None and self.p_size != None:
			for i in range(num):
				temp = []
				for j in range(len(vector[i])):
					u = (float(vector[i][j][0] - self.offset[0]) * xrange_output / xrange_input)
					v = (float(vector[i][j][1] - self.offset[1]) * yrange_output / yrange_input)
					# print(".....")
					print(yrange_output)
					print(yrange_input)
					temp.append((int(u), int(v)))
				mapped.append(temp)
		print('before: ', vector[0])
		print('after: ', mapped[0])
		return mapped

	def string_converter(self, vector):
		""" convert vector into string for publishing """
		all_paths = ''
		num = len(vector)
		for i in range(num):
			path = ''
			num2 = len(vector[i])
			if num2 != 0:
				for j in range(num2):
					if j == (num2 - 1):
						path += str(vector[i][j][0]) + ' ' + str(vector[i][j][1])
					else:
						path += str(vector[i][j][0]) + ' ' + str(vector[i][j][1]) + ' '
				if i == (num - 1):
					all_paths += path
				else:
					all_paths += path + ';'
		return all_paths

	def draw_tasks(self, pixmap):
		self.offset = [(self.t_video_1.size().width() - pixmap.size().width()) / 2, (self.t_video_1.size().height() - pixmap.size().height()) / 2]
		# draw while moving
		if self.draw:
			for i in range(len(self.path) - 1):
				painter = QPainter(pixmap)
				painter.setPen(self.pen)
				painter.drawLine(self.path[i][0] - self.offset[0], self.path[i][1] - self.offset[1], self.path[i + 1][0] - self.offset[0], self.path[i + 1][1] - self.offset[1])
		# draw all paths
		if self.draw_all:
			for i in range(len(self.final_path)):
				for j in range(len(self.final_path[i]) - 1):
					painter = QPainter(pixmap)
					painter.setPen(self.pen)
					if self.draw_all:
						painter.drawLine(self.final_path[i][j][0] - self.offset[0], self.final_path[i][j][1] - self.offset[1], self.final_path[i][j + 1][0] - self.offset[0], self.final_path[i][j + 1][1] - self.offset[1])
						#painter.drawLine(self.final_path[i][j][0], self.final_path[i][j][1], self.final_path[i][j + 1][0], self.final_path[i][j + 1][1])
					else:
						painter.end()
						self.t_video_1.setPixmap(pixmap)
						return
			painter.end()
		self.t_video_1.setPixmap(pixmap)

#################### TASK CALLBACK METHODS ####################
	def cb_task1(self, data):
		if self.mode == 'task':
			pixmap = self.convert_compressed_img(data)
			pixmap = pixmap.scaledToWidth(self.t_video_1.width())
			self.draw_tasks(pixmap)

	def cb_task2(self, data):
		""" cb function for selecting object, only called once """
		return
		if self.mode == 'task':
			txt = data.data.strip()
			if (not self.bbox_ready):
				print('before selecting obj')
				token = txt.split(',')
				self.num = min(4, int(token[0]))
				print('total number of objects: ', self.num)
				self.cls = []
				for i in range(self.num):
					print('adding the {}-th object'.format(i))
					self.cls.append(token[5*i+5])
				self.bbox_ready = True

#################### TASK BUTTON METHODS ####################
	def object_set_id(self, button_id):
		""" set the object name to pick """
		print("the clicked button is", button_id)
		if button_id == 1:
			self.object_id = self.obj_button1.text().strip()
			#self.obj_button1.setChecked(True)
		elif button_id == 2:
			self.object_id = self.obj_button2.text().strip()
			#self.obj_button2.setChecked(True)
		elif button_id == 3:
			self.object_id = self.obj_button3.text().strip()
			#self.obj_button3.setChecked(True)
		elif button_id == 4:
			self.object_id = self.obj_button4.text().strip()
		print('The object id is', self.object_id)

	def task_line(self):
		self.calculate_area = False
		print("Line tasks....")

	def task_area(self):
		self.calculate_area = True
		print("Region tasks...")

	def task_clear(self):
		# reset all paths, stop publishing
		self.draw_all = False
		self.send_path = False
		if self.final_path:
			del self.final_path[:]
		print("Path cleared!!!")

	def task_send(self):
		# publish path to topic
		if self.t_send_button.text() == 'Send':
			print('check if obj id has been set')
			#print(self.object_id)
			if not hasattr(self, 'object_id'):
				# if the object hasn't been selected, pop up warning
				msg = QMessageBox()
				msg.setText('Please select the object')
				msg.setWindowTitle('Forgetting anything?')
				msg.setIcon(QMessageBox.Information)
				msg.setStandardButtons(QMessageBox.Ok)
				ret = msg.exec_()
			elif not self.final_path:
				# if the object hasn't been selected, pop up warning
				msg = QMessageBox()
				msg.setText('Path has not been set yet')
				msg.setWindowTitle('Forgetting anything?')
				msg.setIcon(QMessageBox.Information)
				msg.setStandardButtons(QMessageBox.Ok)
				ret = msg.exec_()
			else:
				self.t_send_button.setText('Confirm?')
		elif self.t_send_button.text() == 'Confirm?':
			# only send if confirmed
			self.send_path = True
			self.t_send_button.setText('sending path...')


### how do we know that the task has completed and move to next stage??
################### do you need both execute and task_send methods/buttons??? #######################

	def task_execute(self):
		self.pub_init.publish('execute')


#################### TASK LAYOUTS ####################
	def object_layout(self):
		print('obj_layout before')
		obj_layout = QVBoxLayout()
		obj_buttons = QButtonGroup()
		print('adding push button {}'.format(self.cls[0]))
		obj_button1 = QPushButton(self.cls[0])
		obj_button1.setCheckable(True)
		obj_button1.setStyleSheet("background-color: rgba(0,255,255,50%)")
		obj_buttons.addButton(obj_button1, 1)
		obj_layout.addWidget(obj_button1)
		if (self.num >= 2):
			print('adding push button {}'.format(self.cls[1]))
			obj_button2 = QPushButton(self.cls[1])
			obj_button2.setCheckable(True)
			obj_button2.setStyleSheet("background-color: rgba(0,255,255,50%)")
			obj_buttons.addButton(obj_button2, 2)
			obj_layout.addWidget(obj_button2)
		if (self.num >= 3):
			print('adding push button {}'.format(self.cls[2]))
			obj_button3 = QPushButton(self.cls[2])
			obj_button3.setCheckable(True)
			obj_button3.setStyleSheet("background-color: rgba(0,255,255,50%)")
			obj_buttons.addButton(obj_button3, 3)
			obj_layout.addWidget(obj_button3)
		if (self.num == 4):
			obj_button4 = QPushButton(self.cls[3])
			obj_button4.setCheckable(True)
			obj_button4.setStyleSheet("background-color: rgba(0,255,255,50%)")
			obj_buttons.addButton(obj_button4)
			obj_layout.addWidget(obj_button4)
		obj_buttons.buttonClicked[int].connect(self.object_set_id)
		return obj_layout

	def task_layout(self):
		layout = QVBoxLayout()
		print('ready to setup object buttons')
		object_layout = self.object_layout()
		button_layout = QHBoxLayout()
		image_layout = QHBoxLayout()

		line_button = QPushButton('Line')
		line_button.clicked.connect(self.task_line)
		line_button.setStyleSheet("background-color: rgba(0,255,255,50%)")

		area_button = QPushButton('Region')
		area_button.clicked.connect(self.task_area)
		area_button.setStyleSheet("background-color: rgba(255,255,0,50%)")

		clear_button = QPushButton('Clear')
		clear_button.clicked.connect(self.task_clear)
		clear_button.setStyleSheet("background-color: rgb(255,0,0,50%)")

		self.t_send_button.clicked.connect(self.task_send)
		self.t_send_button.setStyleSheet("background-color: rgba(0,255,0,50%)")

		execute_button = QPushButton('Execute')
		execute_button.clicked.connect(self.task_execute)
		execute_button.setStyleSheet("background-color: rgba(0,0,0,50%)")

		image_layout.addLayout(object_layout)
		image_layout.addWidget(self.t_video_1)
		button_layout.addWidget(line_button)
		button_layout.addWidget(area_button)
		button_layout.addWidget(clear_button)
		button_layout.addWidget(self.t_send_button)
		button_layout.addWidget(execute_button)
		layout.addLayout(button_layout)
		layout.addLayout(image_layout)
		return layout
################################################################
#
# DETECTING MODE
#
#################### DETECT CALLBACK METHODS ####################
	def cb_detect1(self, data):
		if self.mode == 'detect':
			pixmap = self.convert_compressed_img(data)
			pixmap = pixmap.scaledToWidth(D_IMG_WIDTH)
			self.d_video_1.setPixmap(pixmap)

	def cb_detect2(self, data):
		""" This is to show the imgnet detection of the object """
		if self.mode == 'detect':
			pixmap = self.convert_img(data)
			pixmap = pixmap.scaledToHeight(D_CROP_HEIGHT)
			self.d_video_2.setPixmap(pixmap)

	def cb_detect3(self, data):
		""" This is to show the imgnet detection of the object """
		if self.mode == 'detect':
			pixmap = self.convert_img(data)
			pixmap = pixmap.scaledToHeight(D_CROP_HEIGHT)
			self.d_video_3.setPixmap(pixmap)

#################### DETECT BUTTON METHODS ####################
	def detect_start(self):
		""" Set the state of the button for detection  """
		if self.currently_detecting:
			self.currently_detecting = False
			# publish a ROS message to stop the detection code
			self.pub_detect.publish('stop')
			self.d_detecting_button.setText("Start Detecting")
		else:
			self.currently_detecting = True
			# publish a ROS message to start the learning code, message is the name of the class
			self.pub_detect.publish('start')
			# currently detect, set button to pause if pressed
			self.d_detecting_button.setText("Pause Detecting")

	def detect_stop(self):
		""" conclude the detection stage """
		if self.d_stop_button.text() == 'DONE':
			self.d_stop_button.setText('Confirm?')
		elif self.d_stop_button.text() == 'Confirm?':
			self.pub_detect.publish('end')
			self.d_stop_button.setText('DONE')
			self.update_mode('task')

#################### DETECT LAYOUT ####################
	def detect_layout(self):
		""" set up user interface for the detecting phase """
		self.setWindowTitle('Learning')
		layout = QVBoxLayout()
		button_layout = QHBoxLayout()
		image_layout = QHBoxLayout()
		crop_image_layout = QVBoxLayout()

		self.d_detecting_button.clicked.connect(self.detect_start)
		self.d_detecting_button.setStyleSheet("background-color: rgba(0,255,0,50%)")

		self.d_stop_button.clicked.connect(self.detect_stop)
		self.d_stop_button.setStyleSheet("background-color: rgba(0,0,255,50%)")

		image_layout.addWidget(self.d_video_1)
		crop_image_layout.addWidget(self.d_video_2)
		crop_image_layout.addWidget(self.d_video_3)
		button_layout.addWidget(self.d_detecting_button)
		button_layout.addWidget(self.d_stop_button)
		layout.addLayout(button_layout)
		image_layout.addLayout(crop_image_layout)
		layout.addLayout(image_layout)
		return layout
################################################################
#
# LEARNING MODE
#
#################### LEARN CALLBACK METHODS ####################
	def cb_progress1(self, data):
		if self.mode == 'learn':
			progress = int(data.data)
			print("feature progress is ", progress)
			if progress == 'DONE':
				self.l_done_button.setText('DONE')
				self.update_mode('detect')

	def cb_progress2(self, data):
		if self.mode == 'learn':
			progress = data.data
			print("progress is ", progress)
			if progress == MAX_IMG:
				self.l_learning_button.setText("Learning 100%")
				self.learn_start()
			elif progress < MAX_IMG:
				self.l_learning_button.setText("Learning {:d}%".format(int(float(progress)*100/MAX_IMG)))

	def cb_learn1(self, data):
		if self.mode == 'learn':
			pixmap = self.convert_compressed_img(data)
			pixmap = pixmap.scaledToWidth(self.l_video_1.width())
			self.l_video_1.setPixmap(pixmap)

	def cb_learn2(self, data):
		""" This is to show the cropped view of the object """
		if self.mode == 'learn':
			pixmap = self.convert_img(data)
			pixmap = pixmap.scaledToHeight(D_CROP_HEIGHT)
			self.l_video_2.setPixmap(pixmap)

	def cb_learn3(self, data):
		""" This is to show the heat map of the object """
		if self.mode == 'learn':
			pixmap = self.convert_img(data)
			pixmap = pixmap.scaledToHeight(D_CROP_HEIGHT)
			self.l_video_3.setPixmap(pixmap)

#################### LEARN BUTTON METHODS ####################
	def learn_start(self):
		""" Set the state of the button for collecting training images for a particular object """
		if self.currently_learning: # if already learning, stop learning
			self.currently_learning = False
			# publish a ROS message to stop the learning code
			self.pub_learn.publish("stop")
			self.l_learning_button.setText("Start Learning")
		else:   # if not learning, start learning
			self.pub_init.publish('learn')
			if not self.l_user_input.text():
				# if empty, use default
				self.object_name = DEFAULT_NAME + str(self.num_objects)
			else:
				self.object_name = self.l_user_input.text()
			self.currently_learning = True
			self.l_learning_button.setText("Learning 00%")
			# publish a ROS message to start the learning code, message is the name of the class
			self.pub_learn.publish("start:" + self.object_name + ":" + str(self.num_objects))
			self.num_objects += 1

	def learn_stop(self):
		""" conclude the learning stage """
		if self.l_done_button.text() == 'DONE':
			self.l_done_button.setText('Confirm?')
		elif self.l_done_button.text() == 'Confirm?':
			self.pub_learn.publish('end')

	def learn_delete_object(self):
		if self.mode == 'learn':
			if not self.l_user_input.text():
				# no obj name has been entered, reset
				self.num_objects = 0
				self.pub_learn.publish("reset")
			else:
				self.object_name = self.l_user_input.text()
				self.pub_learn.publish("delete:" + self.object_name)
				self.num_objects -= 1

#################### LEARN LAYOUT ####################
	def learn_layout(self):
		""" set up user interface for the learning phase """
		self.setWindowTitle('Learning')
		layout = QVBoxLayout()
		button_layout = QHBoxLayout()
		image_layout = QHBoxLayout()
		crop_image_layout = QVBoxLayout()

		self.l_learning_button.clicked.connect(self.learn_start)
		self.l_learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")

		self.l_done_button.clicked.connect(self.learn_stop)
		self.l_done_button.setStyleSheet("background-color: rgba(0,0,255,50%)")

		delete_button = QPushButton('DELETE')
		delete_button.clicked.connect(self.learn_delete_object)
		delete_button.setStyleSheet("background-color: rgba(0,255,255,50%)")

		self.l_user_input.setPlaceholderText('Please enter the name of object')
		self.l_user_input.setFocus()

		image_layout.addWidget(self.l_video_1)
		crop_image_layout.addWidget(self.l_video_2)
		crop_image_layout.addWidget(self.l_video_3)

		button_layout.addWidget(self.l_user_input)
		button_layout.addWidget(self.l_learning_button)
		button_layout.addWidget(self.l_done_button)
		button_layout.addWidget(delete_button)

		layout.addLayout(button_layout)
		image_layout.addLayout(crop_image_layout)
		layout.addLayout(image_layout)
		return layout

	def initialize_widgets(self):
		self.stacked_widget = QStackedWidget()
		self.learn_widget = QWidget()
		self.detect_widget = QWidget()
		self.task_widget = QWidget()

		self.l_layout = self.learn_layout()
		self.d_layout = self.detect_layout()
		self.t_layout = self.task_layout()

		self.learn_widget.setLayout(self.l_layout)
		self.detect_widget.setLayout(self.d_layout)
		self.task_widget.setLayout(self.t_layout)

		self.stacked_widget.addWidget(self.learn_widget)
		self.stacked_widget.addWidget(self.detect_widget)
		self.stacked_widget.addWidget(self.task_widget)
		self.main_layout.addWidget(self.stacked_widget)

#################### MODE CHANGE METHODS ####################
	def activate_widget(self, index):
		i = self.stacked_widget.currentIndex()
		if i != index:
			self.stacked_widget.setCurrentIndex(index)
		else:
			print('activating error')

	def change_widget(self):
		if self.mode == 'learn':
			self.activate_widget(0)
		elif self.mode == 'detect':
			self.activate_widget(1)
		elif self.mode == 'task':
			self.activate_widget(2)
		else:
			print('change widget error')

	def update_mode(self, mode):
		self.mode = mode
		self.pub_init.publish(self.mode)
		print('CHANGED TO: %s', self.mode)
		self.change_widget()

#################### IMAGE CONVERSION METHODS ####################
	def convert_img(self, data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError as e:
			print(e)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		return pixmap

	def convert_compressed_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		return pixmap

#################### EVENTS ####################
	def resizeEvent(self, QResizeEvent):
		if self.mode == 'task':
			self.task_clear()

	def mouseMoveEvent(self, QMouseEvent):
		# grab users path and append if path is not to be sent
		if self.mode == 'task':
			self.draw = True
			if not self.send_path:
				# map global position to local image
				pos = QMouseEvent.globalPos()
				#print 'global: ', pos
				pos = self.t_video_1.mapFromGlobal(pos)
				#print 'local: ', pos
				self.path.append([pos.x(), pos.y()])

	def mouseReleaseEvent(self, QMouseEvent):
		if self.mode == 'task':
			self.draw = False
			T = self.path[:]
			# calculate area if needed
			if self.calculate_area:
				area = calculate_trajectory(T)
				if area == None:
					print('Please select an area')
					return
				else:
					temp = []
					for p in area:
						if p != None:
							temp.append(p)
					self.final_path.append(temp)
					self.calculate_area = False
			else:
				self.final_path.append(T)
			del self.path[:]
			self.draw_all = True

	def save_settings(self, plugin_settings, instance_settings):
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		pass
