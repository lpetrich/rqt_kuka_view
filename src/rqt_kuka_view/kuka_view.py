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

import rospy, subprocess, sys, cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from .area_trajectory import calculate_trajectory

class KukaViewWidget(QWidget):
    # initialize widget
    def __init__(self):
        super(KukaViewWidget, self).__init__()
        self.setWindowTitle('Task Interface')
        # set global variables
        self.window_size = self.size()
        self.final_path = []
        self.path = []
        self.send_path = False
        self.calculate_area = False
        # default camera view is for learning
        # if you want to change set the default to True and others to False
        self.learn = True
        self.detect = False
        self.task = False
        # set up pen for drawing paths
        self.pen = QPen(Qt.red)
        self.pen.setWidth(8)
        self.pen.setCapStyle(Qt.RoundCap)
        self.pen.setJoinStyle(Qt.RoundJoin)
        # set up user interface
        self.setup_ui()


    def setup_ui(self):
        # set up user interface
        self.layout = QVBoxLayout()
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
        pixmap = pixmap.scaledToWidth(self.window_size.width() - 30)
        self.video_label.setPixmap(pixmap)

    def view_detection(self, frame):
        # switch view for object detection
        # frame = cv2.flip(frame, 1)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaledToWidth(self.window_size.width() - 30)
        self.video_label.setPixmap(pixmap)

    def view_task(self, frame):
        # frame = cv2.flip(frame, 1)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaledToWidth(self.window_size.width() - 30)
        # draw while moving
        if self.path:
            for i in range(len(self.path) - 1):
                painter = QPainter(pixmap)
                painter.setPen(self.pen)
                y_offset = ((self.window_size.height() - pixmap.size().height()) / 2) - 30
                painter.drawLine(self.path[i][0], self.path[i][1] - y_offset, self.path[i + 1][0], self.path[i + 1][1] - y_offset)
                painter.end()
        # draw all paths
        if self.final_path:
            for i in range(len(self.final_path)):
                for j in range(len(self.final_path[i]) - 1):
                    painter = QPainter(pixmap)
                    painter.setPen(self.pen)
                    y_offset = ((self.window_size.height() - pixmap.size().height()) / 2) - 30
                    painter.drawLine(self.final_path[i][j][0], self.final_path[i][j][1] - y_offset, self.final_path[i][j + 1][0], self.final_path[i][j + 1][1] - y_offset)
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
        if not self.learn:
            self.learn = True
	    self.detect = False
	    self.task = False
            self.progress_bar.reset()

    def enable_detection(self):
        if not self.detect:
            self.detect = True
	    self.learn = False
	    self.task = False

    def enable_tasks(self):
        if not self.task:
            self.task = True
	    self.learn = False
	    self.detect = False
# handle user input
    def clear_path(self):
        # reset all paths, stop publishing
        del self.final_path[:]
        self.send_path = False

    def area_task(self):
        # set area task
        self.calculate_area = True

    def send_tasks(self):
        # publish path to topic /trajectory
        self.send_path = True
        # print self.final_path

    def map_path(self, vector):
        num = len(vector)
        mapped = []
        for i in range(num):
            temp = []
            for j in range(len(vector[i])):
                u = (float(vector[i][j][0]) / 960) * 650
                v = (float(vector[i][j][1]) / 540) * 366            
                temp.append((int(u), int(v)))
            print temp
            mapped.append(temp)
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
        # check if ready to publish 
        if self.send_path:
            tmp = self.final_path[:]
            # print 'before mapping: ', tmp
            mapped_path = self.map_path(tmp)
            all_paths = self.string_converter(mapped_path)
            # print 'mapped: ', all_paths
            # return all_paths
        else:
            return None

    def show_progress(self, progress):
        self.progress_bar.setValue(progress)

# handle events
    def resizeEvent(self, QResizeEvent):
        # reset window_size and clear all paths
        self.window_size = self.size()
        self.clear_path()

    def mouseMoveEvent(self, QMouseEvent):
        # grab users path and append if path is not to be sent
        if not self.send_path:
            # map global position to local image
            pos = self.video_label.mapFromGlobal(QMouseEvent.globalPos())
            self.path.append((pos.x(), pos.y()))
    
    def mouseReleaseEvent(self, QMouseEvent):
        T = self.path[:]
        del self.path[:]
        # calculate area if needed
        if self.calculate_area:
            area = calculate_trajectory(T)
            if area == None:
                print 'Please select an area'
            else:
                temp = []
                for p in area:
                    if p != None:
                        temp.append(p)
                    else:
                        self.final_path.append(temp)
                self.calculate_area = False
        else:
            self.final_path.append(T)

    def save_settings(self, plugin_settings, instance_settings):
        # self._nav_view.save_settings(plugin_settings, instance_settings)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # self._nav_view.restore_settings(plugin_settings, instance_settings)
        pass
