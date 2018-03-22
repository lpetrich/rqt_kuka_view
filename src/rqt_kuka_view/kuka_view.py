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
from geometry_msgs.msg import Point
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from .area_trajectory import calculate_trajectory

DEFAULT_NAME = "tool"
###### size for the image widget
HEIGHT = 180
HEIGHT_DET = 300
WIDTH = 300
######
MAX_IMG = 150

class KukaViewWidget(QWidget):
    # initialize widget
    def __init__(self):
        super(KukaViewWidget, self).__init__()
        self.setWindowTitle('Task Interface')
        # set global variables
        self.window_size = self.size()
        self.final_path = []
        self.path = []
        self.draw = False
        self.draw_all = False
        self.send_path = False
        self.calculate_area = False
        self.p_size = None
        self.i_size = None
        self.offset = [0,0]
        ##### Three stages
        # default camera view is for learning
        # if you want to change set the default to True and others to False
        self.learn = True
        self.detect = False
        self.task = False
        #####
        self.start_learn = False # start/stop learning an object
        self.start_detect = False # start/stop detection, not used for now
        # set up pen for drawing paths
        self.pen = QPen(Qt.red)
        self.pen.setWidth(8)
        self.pen.setCapStyle(Qt.RoundCap)
        self.pen.setJoinStyle(Qt.RoundJoin)
        # set up user interface
        self.init = True
        self.setup_ui()
        self.num_objs = 0 # count the number of objects learned
        self.start_show_crop = True # whether to show cropped image
        # member for layout of buttons to select object
        self.obj_layout = None
        self.selected = False
        self.bbox_ready = False
        self.cls = [] # list of object names

    def setPublisherLearn(self, pub):
        """ set the publisher """
        self._pub_learn = pub

    def setPublisherDet(self, pub):
        """ set the publisher """
        self._pub_det = pub

    def setPublisherTask(self, pub):
        """ set the publisher """
        self._pub_task = pub

    def setPublisherInit(self, pub):
        """ set the publisher for initing """
        self._pub_init = pub

    def flip_learning(self):
        """ Set the state of the button for collecting training images for a particular object """
        if self.start_learn:
            # if already learning, stop learning
            self.start_learn = False
            #self.learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")
            #self.learning_button.setText("Stopped")
            # publish a ROS message to stop the learning code
            self._pub_learn.publish("stop")
        else:
            # if already stopped learning, start learning
            #print("text was ", self.objname_txt.text())
            if not self.objname_txt.text():
                # if empty, use default
                self.objname = DEFAULT_NAME + str(self.num_objs)
            else:
                self.objname = self.objname_txt.text()
            self.start_learn = True
            self.learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")
            self.learning_button.setText("Learning 00%")
            # publish a ROS message to start the learning code
            # message is the name of the class
            self._pub_learn.publish("start:"+self.objname+":"+str(self.num_objs))
            self.num_objs += 1
            # start showing the cropped images
            #self.start_show_crop = True

    def delete_obj(self):
        if self.learn:
            if not self.objname_txt.text():
                #self.objname_txt.setPlaceholderText('Please enter a valid name!')
                # no obj name has been entered, reset
                self.num_objs = 0
                self._pub_learn.publish("reset")
            else:
                self.objname = self.objname_txt.text()
                self._pub_learn.publish("delete:"+self.objname)
                self.num_objs -= 1

    def flip_detection(self):
        """ Set the state of the button for detction  """
        if self.start_detect:
            # if already learning, stop learning
            self.start_detect = False
            self.learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")
            self.learning_button.setText("Stopped")
            # publish a ROS message to stop the detection code
            self._pub_det.publish("stop")
        else:
            # if already stopped learning, start learning
            self.start_detect = True
            self.learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")
            self.learning_button.setText("Detecting")
            # publish a ROS message to start the learning code
            # message is the name of the class
            self._pub_det.publish("start")

    def done_learning(self):
        """ conclude the learning stage """
        if self.next_button.text() == 'DONE':
            self.next_button.setText('Confirm?')
        elif self.next_button.text() == 'Confirm?':
            self._pub_learn.publish("end")
            self.next_button.setText('Ending...')

    def done_detection(self):
        """ conclude the detection stage """
        if self.next_button.text() == 'DONE':
            self.next_button.setText('Confirm?')
        elif self.next_button.text() == 'Confirm?':
            self._pub_det.publish("end")
            self.next_button.setText('Ending')
            self.enable_tasks()

    def dummy(self):
        print("I'm clicked")


    def execute(self):
        self._pub_init.publish('execute')

    def setup_ui_task(self):
        if not self.init:
            print("Re-setup layout")
            # Reconnect the signal
            self.learning_button.setText('Line')
            self.learning_button.clicked.disconnect()
            self.learning_button.clicked.connect(self.line_task)
            self.learning_button.setStyleSheet("background-color: rgba(0,255,255,50%)")

            self.next_button.setText('Region')
            self.next_button.clicked.disconnect()
            self.next_button.clicked.connect(self.area_task)
            self.next_button.setStyleSheet("background-color: rgba(255,255,0,50%)")

            if hasattr(self, 'del_button') and self.del_button is not None:
                # if exists
                print('continue del button')
                self.del_button.setText('Clear')
                self.del_button.clicked.connect(self.clear_path)
                self.del_button.setStyleSheet("background-color: rgba(255,0,0,50%)")
                self.button_layout.addWidget(self.del_button)
            else:
                print('starting del button')
                self.del_button = QPushButton('Clear')
                self.del_button.clicked.connect(self.clear_path)
                self.del_button.setStyleSheet("background-color: rgba(255,0,0,50%)")
                self.button_layout.addWidget(self.del_button)

            # the following two buttons were not available during the detection stage
            self.send_button = QPushButton('Send')
            self.send_button.clicked.connect(self.send_tasks)
            self.send_button.setStyleSheet("background-color: rgba(0,255,0,50%)")
            self.button_layout.addWidget(self.send_button)

            self.cla1_button = QPushButton('Execute')
            self.cla1_button.clicked.connect(self.execute)
            self.cla1_button.setStyleSheet("background-color: rgba(0,0,0,50%)")
            self.button_layout.addWidget(self.cla1_button)

            #if hasattr(self, 'objname_txt'):
            #self.objname_txt.setPlaceholderText('')

            #print('bbox is ready? ', self.bbox_ready)
            print('waiting for bbox', end='')
            while not self.bbox_ready:
                print('.', end='')
            print('ready to setup buttons for selecting objects')
            self.setup_ui_selectobj()
            print('end of setting up task ui')

        else:
            # print ('setup UI ..................................')
            self.layout = QVBoxLayout()
            # group the buttons for different views
            self.button_layout = QHBoxLayout()
            self.image_layout = QHBoxLayout()
            self.sub_img_layout = QVBoxLayout() # for the two small images

            #self.cls_button_layout = QHBoxLayout()#for the five classes buttons

            self.video_label = QLabel()
            self.video_label2 = QLabel() # for the second view for the cropped object
            self.video_label3 = QLabel() # for the third view for the heat map

            self.learning_button = QPushButton('Line')
            self.learning_button.clicked.connect(self.line_task)
            self.learning_button.setStyleSheet("background-color: rgba(0,255,255,50%)")

            self.next_button = QPushButton('Region')
            self.next_button.clicked.connect(self.area_task)
            self.next_button.setStyleSheet("background-color: rgba(255,255,0,50%)")

            #self.rm_button = QPushButton('Remove')
            #self.rm_button.clicked.connect(self.remove_path)
            #self.rm_button.setStyleSheet("background-color: rgba(200,0,0,50%)")

            self.del_button = QPushButton('Clear')
            self.del_button.clicked.connect(self.clear_path)
            self.del_button.setStyleSheet("background-color: rgb(255,0,0,50%)")

            self.send_button = QPushButton('Send')
            self.send_button.clicked.connect(self.send_tasks)
            self.send_button.setStyleSheet("background-color: rgba(0,255,0,50%)")

            self.cla1_button = QPushButton('Execute')
            self.cla1_button.clicked.connect(self.execute)
            self.clas_button.setStyleSheet("background-color: rgba(0,0,0,50%)")


            self.image_layout.addWidget(self.video_label)
            self.sub_img_layout.addWidget(self.video_label2)
            self.sub_img_layout.addWidget(self.video_label3)
            self.button_layout.addWidget(self.learning_button)
            self.button_layout.addWidget(self.next_button)
            #self.button_layout.addWidget(self.rm_button)
            self.button_layout.addWidget(self.del_button)
            self.button_layout.addWidget(self.send_button)
            self.button_layout.addWidget(self.cla1_button)
            self.layout.addLayout(self.button_layout)
            self.image_layout.addLayout(self.sub_img_layout)
            self.layout.addLayout(self.image_layout)

            self.setLayout(self.layout)

    def setup_ui_selectobj(self):
        print('obj_layout before')
        self.obj_layout = QVBoxLayout()
        #self.obj_buttons = QGroupBox("Select the tool")
        self.obj_buttons = QButtonGroup()

        print('adding push button {}'.format(self.cls[0]))
        self.obj_button1 = QPushButton(self.cls[0])
        self.obj_button1.setCheckable(True)
        self.obj_button1.setStyleSheet("background-color: rgba(0,255,255,50%)")
        self.obj_buttons.addButton(self.obj_button1, 1)
        self.obj_layout.addWidget(self.obj_button1)
        if (self.num >= 2):
            print('adding push button {}'.format(self.cls[1]))
            self.obj_button2 = QPushButton(self.cls[1])
            self.obj_button2.setCheckable(True)
            self.obj_button2.setStyleSheet("background-color: rgba(0,255,255,50%)")
            self.obj_buttons.addButton(self.obj_button2, 2)
            self.obj_layout.addWidget(self.obj_button2)
        if (self.num >= 3):
            print('adding push button {}'.format(self.cls[2]))
            self.obj_button3 = QPushButton(self.cls[2])
            self.obj_button3.setCheckable(True)
            self.obj_button3.setStyleSheet("background-color: rgba(0,255,255,50%)")
            self.obj_buttons.addButton(self.obj_button3, 3)
            self.obj_layout.addWidget(self.obj_button3)
        if (self.num == 4):
            self.obj_button4 = QPushButton(self.cls[3])
            self.obj_button4.setCheckable(True)
            self.obj_button4.setStyleSheet("background-color: rgba(0,255,255,50%)")
            self.obj_buttons.addButton(self.obj_button4)
            self.obj_layout.addWidget(self.obj_button4)

        self.obj_buttons.buttonClicked[int].connect(self.set_obj_id)
        self.image_layout.addLayout(self.obj_layout)

    def select_obj(self, data):
        """ cb function for selecting object, only called once """
        if (not self.bbox_ready) and self.task:
            print('before selecting obj')
            token = data.split(',')
            self.num = min(4, int(token[0]))
            print('total number of objects: ', self.num)
            self.cls = []
            for i in range(self.num):
                print('adding the {}-th object'.format(i))
                self.cls.append(token[5*i+5])
            self.bbox_ready = True

    def set_obj_id(self, button_id):
        """ set the object name to pick """
        print("the clicked button is", button_id)
        if button_id == 1:
            self.obj_id = self.obj_button1.text().strip()
            #self.obj_button1.setChecked(True)
        elif button_id == 2:
            self.obj_id = self.obj_button2.text().strip()
            #self.obj_button2.setChecked(True)
        elif button_id == 3:
            self.obj_id = self.obj_button3.text().strip()
            #self.obj_button3.setChecked(True)
        elif button_id == 4:
            self.obj_id = self.obj_button4.text().strip()
        print('The object id is', self.obj_id)

    def setup_ui_detect(self):
        """ set up user interface for the detect phase """
        if not self.init:
            print("Re-setup layout")
            print("Before removing ")
            #self.objname_txt.setPlaceholderText('')
            self.button_layout.removeWidget(self.objname_txt) # remove the delete button
            self.objname_txt.setParent(None)
            self.objname_txt = None
            self.del_button.clicked.disconnect()
            self.button_layout.removeWidget(self.del_button) # remove the delete button
            self.del_button.setParent(None)
            #self.del_button.deleteLater()
            #self.del_button = None
            print("After removing ")

            # Reconnect the signal
            self.learning_button.clicked.disconnect()
            self.learning_button.setText('Loading detection')
            self.learning_button.clicked.connect(self.flip_detection)
            self.learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")

            self.next_button.clicked.disconnect()
            self.next_button.setText('DONE')
            self.next_button.clicked.connect(self.done_detection)
            self.next_button.setStyleSheet("background-color: rgba(0,0,255,50%)")
            print("finished reconnecting the signals")

        else:
            print("Initing the layout for detection")
            self.layout = QVBoxLayout()
            # group the buttons for different views
            self.button_layout = QHBoxLayout()
            self.image_layout = QHBoxLayout()
            self.sub_img_layout = QVBoxLayout() # for the two small images

            self.video_label = QLabel()
            self.video_label2 = QLabel() # for the second view for the cropped object
            self.video_label3 = QLabel() # for the third view for the heat map

            self.learning_button = QPushButton('Loading detection')
            self.learning_button.clicked.connect(self.flip_detection)
            self.learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")

            self.next_button = QPushButton('DONE')
            self.next_button.clicked.connect(self.done_detection)
            self.next_button.setStyleSheet("background-color: rgba(0,0,255,50%)")

            self.image_layout.addWidget(self.video_label)
            self.sub_img_layout.addWidget(self.video_label2)
            self.sub_img_layout.addWidget(self.video_label3)
            self.button_layout.addWidget(self.learning_button)
            self.button_layout.addWidget(self.next_button)
            self.layout.addLayout(self.button_layout)
            self.image_layout.addLayout(self.sub_img_layout)
            self.layout.addLayout(self.image_layout)

            self.setLayout(self.layout)
            self.init = False

    def setup_ui_learn(self):
        """ set up user interface for the learning phase """
        self.layout = QVBoxLayout()
        # group the buttons for different views
        self.button_layout = QHBoxLayout()
        self.image_layout = QHBoxLayout()
        self.sub_img_layout = QVBoxLayout() # for the two small images

        self.video_label = QLabel()
        self.video_label2 = QLabel() # for the second view for the cropped object
        self.video_label3 = QLabel() # for the third view for the heat map

        self.learning_button = QPushButton('Start learning')
        self.learning_button.clicked.connect(self.flip_learning)
        self.learning_button.setStyleSheet("background-color: rgba(0,255,0,50%)")

        self.next_button = QPushButton('DONE')
        self.next_button.clicked.connect(self.done_learning)
        self.next_button.setStyleSheet("background-color: rgba(0,0,255,50%)")

        self.del_button = QPushButton('DELETE')
        self.del_button.clicked.connect(self.delete_obj)
        self.del_button.setStyleSheet("background-color: rgba(0,255,255,50%)")

        self.objname_txt = QLineEdit()
        self.objname_txt.setPlaceholderText('Please enter the name of object')
        self.objname_txt.setFocus()

        self.image_layout.addWidget(self.video_label)
        self.sub_img_layout.addWidget(self.video_label2)
        self.sub_img_layout.addWidget(self.video_label3)
        self.button_layout.addWidget(self.objname_txt)
        self.button_layout.addWidget(self.learning_button)
        self.button_layout.addWidget(self.next_button)
        self.button_layout.addWidget(self.del_button)
        self.layout.addLayout(self.button_layout)
        self.image_layout.addLayout(self.sub_img_layout)
        self.layout.addLayout(self.image_layout)

        self.setLayout(self.layout)
        ## the following is on the very first module
        self.init = False

    def setup_ui(self):
        if self.learn:
            self.setup_ui_learn()
        elif self.detect:
            self.setup_ui_detect()
        elif self.task:
            self.setup_ui_task()

    def init_modules(self):
        if self.learn:
            self.enable_learning()
        elif self.detect:
            self.enable_detection()
        elif self.task:
            self.enable_tasks()

    def setup_ui_original(self):
        """ The original (old) function for all the UI components"""
        # set up user interface
        self.layout = QVBoxLayout()
        # group the buttons for different views
        self.button_layout = QHBoxLayout()
        self.image_layout = QHBoxLayout()
        self.progress_layout = QHBoxLayout()

        self.video_label = QLabel()

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)

        self.area_button = QPushButton('set area task')
        self.area_button.clicked.connect(self.area_task)

        self.clear_button = QPushButton('clear tasks')
        self.clear_button.clicked.connect(self.clear_path)

        self.move_button = QPushButton('send tasks')
        self.move_button.clicked.connect(self.send_tasks)

        self.learning_button = QPushButton('learn new object')
        self.learning_button.clicked.connect(self.enable_learning)

        self.task_button = QPushButton('set new tasks')
        self.task_button.clicked.connect(self.enable_tasks)

        self.detection_button = QPushButton('detect objects')
        self.detection_button.clicked.connect(self.enable_detection)
	# rearrange below to change order of buttons
        self.button_layout.addWidget(self.learning_button)
        self.button_layout.addWidget(self.detection_button)
        self.button_layout.addWidget(self.task_button)
        self.button_layout.addWidget(self.area_button)
        self.button_layout.addWidget(self.clear_button)
        self.button_layout.addWidget(self.move_button)

        self.progress_layout.addWidget(self.progress_bar)
        self.image_layout.addWidget(self.video_label)
        self.layout.addLayout(self.button_layout)
        self.layout.addLayout(self.image_layout)
        self.layout.addLayout(self.progress_layout)

        self.setLayout(self.layout)

    # handle different view events
    def view_learning(self, frame):
        # learning task enabled, switch view to firewire
        # frame = cv2.flip(frame, 1)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        #pixmap = pixmap.scaledToWidth(self.window_size.width() - 30)
        #pixmap = pixmap.scaledToWidth(self.window_size.width())
        pixmap = pixmap.scaledToWidth(self.video_label.width())
        self.video_label.setPixmap(pixmap)

    def clear_pixmap(self):
        """ clear the cropped image and the heatmap """
        pixmap = QPixmap(WIDTH, HEIGHT_DET)
        pixmap.fill(Qt.transparent)
        self.video_label2.setPixmap(pixmap)
        self.video_label3.setPixmap(pixmap)

    def show_crop(self, frame):
        """ This is to show the cropped view of the object """
        if self.start_show_crop:
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = image.scaled(WIDTH,HEIGHT)
            pixmap = QPixmap.fromImage(image)
            #pixmap = pixmap.scaledToWidth(self.video_label3.width())
            #pixmap = pixmap.scaledToWidth(200)
            self.video_label3.setPixmap(pixmap)

    def show_heatmap(self, frame):
        """ This is to show the heat map of the object """
        if self.start_show_crop:
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = image.scaled(WIDTH, HEIGHT)
            pixmap = QPixmap.fromImage(image)
            #pixmap = pixmap.scaledToWidth(self.video_label3.width())
            self.video_label2.setPixmap(pixmap)

    def show_det_before(self, frame):
        """ This is to show the imgnet detection of the object """
        if not self.learn:
            if not self.start_detect:
                self.start_detect = True
                if self.detect: # we might be in the task stage instead since we expect the detection module to deep running to provide bbox info
                    self.learning_button.setText('Detecting')
                    self.learning_button.setStyleSheet("background-color: rgba(255,0,0,50%)")
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = image.scaled(WIDTH, HEIGHT_DET)
            pixmap = QPixmap.fromImage(image)
            self.video_label3.setPixmap(pixmap)

    def show_det_after(self, frame):
        """ This is to show the imgnet detection of the object """
        if not self.learn:
            image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
            image = image.scaled(WIDTH, HEIGHT_DET)
            pixmap = QPixmap.fromImage(image)
            self.video_label2.setPixmap(pixmap)

    def view_detection(self, frame):
        # switch view for object detection
        # frame = cv2.flip(frame, 1)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaledToWidth(self.video_label.width())
        self.video_label.setPixmap(pixmap)

    def view_task(self, frame):
        # frame = cv2.flip(frame, 1)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaledToWidth(self.video_label.width()) #window_size.width() - 30)
        self.offset = [(self.video_label.size().width() - pixmap.size().width()) / 2, (self.video_label.size().height() - pixmap.size().height()) / 2]
        self.p_size = pixmap.size()
        self.i_size = image.size()
        # draw while moving
        if self.draw:
            for i in range(len(self.path) - 1):
                painter = QPainter(pixmap)
                painter.setPen(self.pen)
		if self.draw:
                    painter.drawLine(self.path[i][0] - self.offset[0], self.path[i][1] - self.offset[1], self.path[i + 1][0] - self.offset[0], self.path[i + 1][1] - self.offset[1])
		else:
		    painter.end()
		    break
		painter.end()
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
			self.video_label.setPixmap(pixmap)
			return
                    painter.end()
        self.video_label.setPixmap(pixmap)

    # check which image to show
    def check_learning(self, image):
        if self.learn:
            self.view_learning(image)

    def check_detection(self, image):
        if self.detect:
            self.view_detection(image)

    def check_task(self, image):
        if self.task:
            self.view_task(image)

    # enable appropriate image
    def enable_learning(self):
        #if not self.learn:
        self.learn = True
        self.detect = False
        self.task = False
        self._pub_init.publish('learn')
        #self.setup_ui()

    def enable_detection(self):
        if not self.detect:
            self.detect = True
            self.learn = False
            self.task = False
            self.setup_ui_detect()
        self._pub_init.publish('detect')

    def enable_tasks(self):
        if not self.task:
            self.task = True
            self.learn = False
            self.detect = False
            self.setup_ui_task()


    def remove_path(self):
        #remove the last drawn path
        print("list..........")
        print(len(self.final_path))
        self.draw = False
        if self.final_path:
            del self.final_path[-1]
        print("The last path is removed!!!")

    # handle user input
    def clear_path(self):
        # reset all paths, stop publishing
        self.draw_all = False
        self.send_path = False
        if self.final_path:
            del self.final_path[:]
        print("Path cleared!!!")

    def line_task(self):
        #set area task
        self.calculate_area = False
        print("Line tasks....")

    def area_task(self):
        # set area task
        self.calculate_area = True
        print("Region tasks...")

    def send_tasks(self):
        # publish path to topic
        if self.send_button.text() == 'Send':
            print('check if obj id has been set')
            #print(self.obj_id)
            if not hasattr(self, 'obj_id'):
                # if the object hasn't been selected, pop up warning
                msg = QMessageBox()
                #print('creating message box')
                msg.setText('Please select the object')
                msg.setWindowTitle('Forgot anything?')
                msg.setIcon(QMessageBox.Information)
                msg.setStandardButtons(QMessageBox.Ok)
                ret = msg.exec_()
            elif not self.final_path:
                # if the object hasn't been selected, pop up warning
                msg = QMessageBox()
                msg.setText('Has not set path yet')
                msg.setWindowTitle('Forgot anything?')
                msg.setIcon(QMessageBox.Information)
                msg.setStandardButtons(QMessageBox.Ok)
                ret = msg.exec_()
            else:
                self.send_button.setText('Confirm?')
        elif self.send_button.text() == 'Confirm?':
            # only send if confirmed
            self.send_path = True
            self.send_button.setText('Send')

    def map_path(self, vector):
        num = len(vector)
    #	xrange_input = self.video_label.size().width()
        xrange_input = self.p_size.width()
    	xrange_output = self.i_size.width()
   # 	yrange_input = self.video_label.size().height()
        yrange_input = self.p_size.height()
    	yrange_output = self.i_size.height()
        mapped = []
    	if self.i_size != None and self.p_size != None:
    		for i in range(num):
    		    temp = []
    		    for j in range(len(vector[i])):
    			u = (float(vector[i][j][0] - self.offset[0]) * xrange_output / xrange_input)
    			v = (float(vector[i][j][1] - self.offset[1]) * yrange_output / yrange_input)
                        print(".....")
                        print(yrange_output)
                        print(yrange_input)
    		        temp.append((int(u), int(v)))
    		    mapped.append(temp)
    	print('before: ', vector[0])
    	print('after: ', mapped[0])
        return mapped

    def string_converter(self, vector):
        # convert vector into string for publishing
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

    def check_if_path_ready(self):
        """ check if ready to publish """
        if self.send_path:
            self._pub_init.publish('grasp:'+self.obj_id)
            tmp = self.final_path[:]
            #print 'before mapping: ', tmp
            mapped_path = self.map_path(tmp)
            all_paths = self.string_converter(mapped_path)
            #print 'mapped: ', all_paths
            self.send_path = False
            return all_paths
        else:
            return None

    def show_progress(self, progress):
        print("progress is ", progress)
        if progress == MAX_IMG:
            self.learning_button.setText("Learning 100%")
            self.flip_learning()
        elif progress < MAX_IMG:
            self.learning_button.setText("Learning {:d}%".format(int(float(progress)*100/MAX_IMG)))

    def show_feat_progress(self, progress):
        print("feature progress is ", progress)
        if progress == 'DONE':
            self.next_button.setText('End')
            self.enable_detection()
            # clear the heatmap and the cropped image
            self.clear_pixmap()

# handle events
    def resizeEvent(self, QResizeEvent):
        # reset window_size and clear all paths
        self.window_size = self.size()
        #pixmap = pixmap.scaledToWidth(self.window_size.width() - 30)
        #self.offset = self.video_label.size().width() - (self.window_size.width() - 30)
        print('offset: ', self.offset)
        print('video label: ', self.video_label.size())
        print('window size: ', self.window_size)
        print('pixmap size: ',self.p_size)
        print('img size: ', self.i_size)
        if self.task:
            self.clear_path()

    def mouseMoveEvent(self, QMouseEvent):
        # grab users path and append if path is not to be sent
        self.draw = True
        if not self.send_path:
            # map global position to local image
	    pos = QMouseEvent.globalPos()
	    #print 'global: ', pos
            pos = self.video_label.mapFromGlobal(pos)
	    #print 'local: ', pos
            self.path.append([pos.x(), pos.y()])

    def mouseReleaseEvent(self, QMouseEvent):
        print('yoffset: ', self.offset)
        print('video label: ', self.video_label.size())
        print('window size: ', self.window_size)
        print('pixmap size: ',self.p_size)
        print('img size: ', self.i_size)
        self.draw = False
        T = self.path[:]
        # calculate area if needed
        if self.calculate_area:
            area = calculate_trajectory(T)
            if area == None:
                print('Please select an area')
		self.send_path = False
		return
            else:
                temp = []
                for p in area:
                    if p != None:
                        temp.append(p)
                    else:
                        self.final_path.append(temp)
           #     self.calculate_area = False
        else:
            self.final_path.append(T)
        del self.path[:]
        self.draw_all = True

    def save_settings(self, plugin_settings, instance_settings):
        # self._nav_view.save_settings(plugin_settings, instance_settings)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # self._nav_view.restore_settings(plugin_settings, instance_settings)
        pass
