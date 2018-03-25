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

from __future__ import print_function # have to be the first line
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
#L_IMG_WIDTH = 300
#L_CROP_HEIGHT = 180
MAX_IMG = 100
DEFAULT_NAME = "tool"
##### DETECT #####
D_IMG_WIDTH = 300
D_CROP_HEIGHT = 180
D_CROP_WIDTH = 300
##### TASK #####
T_IMG_WIDTH = 600
T_SUBIMG_WIDTH = 350


class KukaViewWidget(QWidget):
    def __init__(self):
        super(KukaViewWidget, self).__init__()
        self.setWindowTitle('KUKA Interface')
        self.main_layout = QVBoxLayout()
        self.mode = 'empty'

        ##### Define the custom signals and slots for bridging with ROS callbacks #####
        # such that ros callbacks will emit signal instead directly manipulating UI componets
        #
        # This is due to the following guidelines from http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin
        # Due to restrictions in Qt, you cannot manipulate Qt widgets directly within ROS callbacks,
        # because they are running in a different thread.
        # In the ROS callback you can:
        #   emit a Qt signal (which will bridge across the threads) and manipulate the widgets in the receiving slot
        self.connect(self, SIGNAL("Image-learn arrived"), self.show_learn)
        self.connect(self, SIGNAL("Image-detect arrived"), self.show_detect)
        self.connect(self, SIGNAL("Image-task arrived"), self.show_task)
        self.connect(self, SIGNAL("Crop arrived"), self.show_crop)
        self.connect(self, SIGNAL("Heatmap arrived"), self.show_heatmap)
        self.connect(self, SIGNAL("Show-det-before"), self.show_det_before)
        self.connect(self, SIGNAL("Show-det-after"), self.show_det_after)
        self.connect(self, SIGNAL("Show-task-after"), self.show_task_after)
        self.connect(self, SIGNAL("Show-task-path"), self.show_task_path)
        self.connect(self, SIGNAL("Add-obj-list"), self.add_obj_list)
        self.connect(self, SIGNAL("Progress_learn arrived"), self.show_progress_learn)
        self.connect(self, SIGNAL("Progress_feature arrived"), self.show_progress_feature)

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
        self.t_video_2 = QLabel()
        self.t_video_3 = QLabel()
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
        self.p_size = None
        self.i_size = None
        # the object selection layout
        self.object_id = None
        self.obj_list_ready = False
        self.obj_buttons = None
        self.obj_button1 = None
        self.obj_button2 = None
        self.obj_button3 = None
        self.obj_button4 = None
        self.num_bbox = 4
        self.cls = ['', '', '', ''] # list of object names
        ##### SHARED VARIABLES #####
        self.bridge = CvBridge()
        self.initialize_widgets()
        self.setLayout(self.main_layout)

        ######################## setup publishers and subscribers ########################
        ##### LEARN #####
        self.pub_learn = rospy.Publisher('/command_learn', String, queue_size=10)
        self.sub_learn1 = rospy.Subscriber('/cam1/camera/image_raw/compressed',  CompressedImage, self.cb_learn1, queue_size = 1)
        self.sub_learn2 = rospy.Subscriber('/map', Image, self.cb_learn2,  queue_size = 1)
        self.sub_learn3 = rospy.Subscriber('/crop', Image, self.cb_learn3, queue_size = 1)
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
        self.sub_task2 = rospy.Subscriber('/bbox', String, self.cb_task2,  queue_size = 1)
        self.sub_task3 = rospy.Subscriber('/path_image', Image, self.cb_task3,  queue_size = 1)
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
            self.t_send_button.setText('Send')
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
        self.p_size = pixmap.size()
        # draw while moving
        if self.draw:
            for i in range(len(self.path) - 1):
                painter = QPainter(pixmap)
                painter.setPen(self.pen)
                painter.drawLine(self.path[i][0] - self.offset[0], self.path[i][1] - self.offset[1], self.path[i + 1][0] - self.offset[0], self.path[i + 1][1] - self.offset[1])
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
                        self.t_video_1.setPixmap(pixmap)
                        return
                    painter.end()
        self.t_video_1.setPixmap(pixmap)

#################### ROS, TASK CALLBACK METHODS ####################
    def cb_task1(self, data):
        if self.mode == 'task':
            self.signal_video_task(data)

    def cb_task2(self, data):
        """ cb function for selecting object, only called once """
        if self.mode == 'task':
            if not self.obj_list_ready:
                self.signal_add_obj_list(data)

    def cb_task3(self, data):
        """ cb function for selecting object, only called once """
        if self.mode == 'task':
            self.signal_video_task_path(data)

    ## custom signals that update the UI from ROS cb
    def signal_video_task(self, data):
        self.emit(SIGNAL("Image-task arrived"), data)

    def signal_add_obj_list(self, data):
        self.emit(SIGNAL("Add-obj-list"), data)

    ## slots that update the UI
    def show_task(self, data):
        pixmap = self.convert_compressed_img(data)
        self.i_size = pixmap.size()
        #pixmap = pixmap.scaledToWidth(self.t_video_1.width())
        pixmap = pixmap.scaledToWidth(T_IMG_WIDTH)
        self.draw_tasks(pixmap)

    def add_obj_list(self, data):
        """ Fill the object list and update UI """
        print('before selecting obj')
        txt = data.data.strip()
        token = txt.split(',')
        self.num_bbox = min(4, int(token[0]))
        print('total number of objects: ', self.num_bbox)
        self.cls = ['', '', '', '']
        for i in range(self.num_bbox):
            print('adding the {}-th object'.format(i))
            self.cls[i] = token[5*i+5]
        self.cls.sort(reverse=True)
        #self.obj_list_ready = True
        self.update_object_layout()

#################### TASK BUTTON METHODS ####################
    def set_object_id(self, button_id):
        """ set the object name to pick """
        button = self.obj_buttons.button(button_id)
        self.object_id = button.text().strip()
        print('The object id is', self.object_id)
        if self.object_id == '':
            msg = QMessageBox()
            msg.setText('Please select a valid object')
            msg.setWindowTitle('Hmm ...')
            msg.setIcon(QMessageBox.Information)
            msg.setStandardButtons(QMessageBox.Ok)
            ret = msg.exec_()
            # uncheck the button.
            # the following was to get around a bug in Qt
            self.obj_buttons.setExclusive(False)
            button.setChecked(False)
            self.obj_buttons.setExclusive(True)

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
            if (not self.object_id) or (self.object_id == ''):
                # if the object hasn't been selected, pop up warning
                msg = QMessageBox()
                msg.setText('Please select the object')
                msg.setWindowTitle('Forgetting anything?')
                msg.setIcon(QMessageBox.Information)
                msg.setStandardButtons(QMessageBox.Ok)
                ret = msg.exec_()
            elif self.object_id not in self.cls:
                # if the object selected is not in the list anymore
                msg = QMessageBox()
                msg.setText('Object not there anymore. Please re-select the object')
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

    def task_execute(self):
        self.pub_init.publish('execute')

#################### TASK LAYOUTS ####################
    def update_object_layout(self):
        print('Updating obj_layout ...')
        # update the txt
        self.obj_button1.setText(self.cls[0])
        self.obj_button2.setText(self.cls[1])
        self.obj_button3.setText(self.cls[2])
        self.obj_button4.setText(self.cls[3])

    def object_layout(self):
        print('Setting up obj_layout ...')
        obj_layout = QVBoxLayout()
        self.obj_buttons = QButtonGroup()
        print('adding push button {}'.format(self.cls[0]))
        self.obj_button1 = QPushButton(self.cls[0])
        self.obj_button1.setCheckable(True)
        self.obj_button1.setStyleSheet("background-color: rgba(0,255,255,50%)")
        self.obj_buttons.addButton(self.obj_button1, 1)
        obj_layout.addWidget(self.obj_button1)
        print('adding push button {}'.format(self.cls[1]))
        self.obj_button2 = QPushButton(self.cls[1])
        self.obj_button2.setCheckable(True)
        self.obj_button2.setStyleSheet("background-color: rgba(0,255,255,50%)")
        self.obj_buttons.addButton(self.obj_button2, 2)
        obj_layout.addWidget(self.obj_button2)
        print('adding push button {}'.format(self.cls[2]))
        self.obj_button3 = QPushButton(self.cls[2])
        self.obj_button3.setCheckable(True)
        self.obj_button3.setStyleSheet("background-color: rgba(0,255,255,50%)")
        self.obj_buttons.addButton(self.obj_button3, 3)
        obj_layout.addWidget(self.obj_button3)
        print('adding push button {}'.format(self.cls[3]))
        self.obj_button4 = QPushButton(self.cls[3])
        self.obj_button4.setCheckable(True)
        self.obj_button4.setStyleSheet("background-color: rgba(0,255,255,50%)")
        self.obj_buttons.addButton(self.obj_button4, 4)
        obj_layout.addWidget(self.obj_button4)
        self.obj_buttons.buttonClicked[int].connect(self.set_object_id)
        #self.obj_buttons.buttonClicked[QAbstractButton].connect(self.set_object_id)
        return obj_layout

    def task_layout(self):
        self.setWindowTitle('Task')
        layout = QVBoxLayout()
        button_layout = QHBoxLayout()
        image_layout = QHBoxLayout()
        object_layout = self.object_layout()
        crop_image_layout = QVBoxLayout()

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
        execute_button.setStyleSheet("background-color: rgba(0,0,0,50%) ; color: white")

        image_layout.addWidget(self.t_video_1)
        crop_image_layout.addWidget(self.t_video_2)
        crop_image_layout.addWidget(self.t_video_3)
        image_layout.addLayout(crop_image_layout)
        button_layout.addWidget(line_button)
        button_layout.addWidget(area_button)
        button_layout.addWidget(clear_button)
        button_layout.addWidget(self.t_send_button)
        button_layout.addWidget(execute_button)
        layout.addLayout(button_layout)
        image_layout.addLayout(object_layout)
        layout.addLayout(image_layout)
        return layout


################################################################
#
# DETECTING MODE
#
#################### DETECT CALLBACK METHODS ####################
    def cb_detect1(self, data):
        if self.mode == 'detect':
            self.signal_video_detect(data)

    def cb_detect2(self, data):
        """ This is to show the new detection of the object """
        if self.mode == 'detect':
            self.signal_video_det_after(data)
        elif self.mode == 'task':
            self.signal_video_task_after(data)

    def cb_detect3(self, data):
        """ This is to show the imgnet detection of the object """
        if self.mode == 'detect':
            self.signal_video_det_before(data)

    ## custom signals that update the UI from ROS cb
    def signal_video_detect(self, data):
        self.emit(SIGNAL("Image-detect arrived"), data)

    def signal_video_det_after(self, data):
        self.emit(SIGNAL("Show-det-after"), data)

    def signal_video_det_before(self, data):
        self.emit(SIGNAL("Show-det-before"), data)

    def signal_video_task_after(self, data):
        self.emit(SIGNAL("Show-task-after"), data)

    def signal_video_task_path(self, data):
        self.emit(SIGNAL("Show-task-path"), data)

    ## slots that update the UI
    def show_detect(self, data):
        pixmap = self.convert_compressed_img(data)
        pixmap = pixmap.scaledToWidth(self.d_video_1.width())
        self.d_video_1.setPixmap(pixmap)

    def show_det_after(self, data):
        pixmap = self.convert_img(data)
        pixmap = pixmap.scaledToWidth(D_IMG_WIDTH)
        self.d_video_2.setPixmap(pixmap)
        if not self.currently_detecting:
            # currently detect, set button to pause if pressed
            self.d_detecting_button.setText("Pause Detecting")
            self.d_detecting_button.setStyleSheet("background-color: rgba(255,0,0,50%)")

    def show_det_before(self, data):
        pixmap = self.convert_img(data)
        pixmap = pixmap.scaledToWidth(D_IMG_WIDTH)
        self.d_video_3.setPixmap(pixmap)

    def show_task_after(self, data):
        pixmap = self.convert_img(data)
        pixmap = pixmap.scaledToWidth(T_SUBIMG_WIDTH)
        self.t_video_2.setPixmap(pixmap)

    def show_task_path(self, data):
        pixmap = self.convert_img(data)
        pixmap = pixmap.scaledToWidth(T_SUBIMG_WIDTH)
        self.t_video_3.setPixmap(pixmap)


#################### DETECT BUTTON METHODS ####################
    def detect_start(self):
        """ Set the state of the button for detection  """
        if self.currently_detecting:
            self.currently_detecting = False
            # publish a ROS message to stop the detection code
            self.pub_detect.publish('stop')
            self.d_detecting_button.setText("Start Detecting")
            self.d_detecting_button.setStyleSheet("background-color: rgba(0,255,0,50%)")
        else:
            self.currently_detecting = True
            # publish a ROS message to start the detection code, message is the name of the class
            self.pub_detect.publish('start')
            # currently detect, set button to pause if pressed
            self.d_detecting_button.setText("Pause Detecting")
            self.d_detecting_button.setStyleSheet("background-color: rgba(255,0,0,50%)")

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
        self.setWindowTitle('Detection')
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
#################### ROS: LEARN CALLBACK METHODS ####################
    def cb_progress1(self, data):
        if self.mode == 'learn':
            self.signal_progress_feature(data)

    def cb_progress2(self, data):
        if self.mode == 'learn':
            self.signal_progress_learn(data)

    def cb_learn1(self, data):
        """ This is to show the main view of the learning phase """
        if self.mode == 'learn':
            self.signal_video_learn(data)

    def cb_learn2(self, data):
        """ This is to show the heat map of the object """
        if self.mode == 'learn':
            self.signal_video_heatmap(data)

    def cb_learn3(self, data):
        """ This is to show the cropped view of the object """
        if self.mode == 'learn':
            self.signal_video_crop(data)

    ## custom signals that update the UI from ROS cb
    def signal_video_learn(self, data):
        self.emit(SIGNAL("Image-learn arrived"), data)

    def signal_video_crop(self, data):
        self.emit(SIGNAL("Crop arrived"), data)

    def signal_video_heatmap(self, data):
        self.emit(SIGNAL("Heatmap arrived"), data)

    def signal_progress_learn(self, data):
        self.emit(SIGNAL("Progress_learn arrived"), data)

    def signal_progress_feature(self, data):
        self.emit(SIGNAL("Progress_feature arrived"), data)

    ## slots that update the UI
    def show_learn(self, data):
        pixmap = self.convert_compressed_img(data)
        pixmap = pixmap.scaledToWidth(self.l_video_1.width())
        self.l_video_1.setPixmap(pixmap)

    def show_heatmap(self, data):
        pixmap = self.convert_img(data)
        pixmap = pixmap.scaled(D_CROP_WIDTH, D_CROP_HEIGHT)
        self.l_video_2.setPixmap(pixmap)

    def show_crop(self, data):
        pixmap = self.convert_img(data)
        pixmap = pixmap.scaled(D_CROP_WIDTH, D_CROP_HEIGHT)
        self.l_video_3.setPixmap(pixmap)

    def show_progress_learn(self, data):
        progress = int(data.data)
        print("learning progress is ", progress)
        if progress == MAX_IMG:
            self.l_learning_button.setText("Learning 100%")
            self.learn_start()
        elif progress < MAX_IMG:
            self.l_learning_button.setText("Learning {:d}%".format(int(float(progress)*100/MAX_IMG)))

    def show_progress_feature(self, data):
        progress = data.data
        print("feature progress is ", progress)
        if progress == 'DONE':
            self.l_done_button.setText('DONE')
            self.update_mode('detect')

#################### LEARN BUTTON METHODS ####################
    def learn_start(self):
        """ Set the state of the button for collecting training images for a particular object """
        if self.currently_learning: # if already learning, stop learning
            self.currently_learning = False
            # publish a ROS message to stop the learning code
            self.pub_learn.publish("stop")
            self.l_learning_button.setText("Start Learning")
        else:   # if not learning, start learning
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
        self.empty_widget = QWidget() # empty widget to show in the beginning

        self.l_layout = self.learn_layout()
        self.d_layout = self.detect_layout()
        self.t_layout = self.task_layout()

        self.learn_widget.setLayout(self.l_layout)
        self.detect_widget.setLayout(self.d_layout)
        self.task_widget.setLayout(self.t_layout)

        self.stacked_widget.addWidget(self.empty_widget)
        self.stacked_widget.addWidget(self.learn_widget)
        self.stacked_widget.addWidget(self.detect_widget)
        self.stacked_widget.addWidget(self.task_widget)
        self.main_layout.addWidget(self.stacked_widget)
        self.setFocusPolicy(Qt.StrongFocus)

#################### MODE CHANGE METHODS ####################
    def activate_widget(self, index):
        i = self.stacked_widget.currentIndex()
        if i != index:
            self.stacked_widget.setCurrentIndex(index)
        else:
            print('activating error')

    def change_widget(self):
        if self.mode == 'learn':
            self.activate_widget(1)
        elif self.mode == 'detect':
            self.activate_widget(2)
        elif self.mode == 'task':
            self.obj_list_ready = False
            self.activate_widget(3)
        elif self.mode == 'empty':
            self.activate_widget(0)
        else:
            print('change widget error')

    def update_mode(self, mode):
        self.mode = mode
        self.pub_init.publish(self.mode)
        print('CHANGED TO: {}'.format(self.mode))
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

    def keyPressEvent(self, key_event):
        """ grab the key to switch the views  """
        k = key_event.key()
        print("The registered key was", k)
        if k == Qt.Key_L:
            self.update_mode("learn")
        elif k == Qt.Key_D:
            self.update_mode("detect")
        elif k == Qt.Key_T:
            self.update_mode("task")
        elif k == Qt.Key_E:
            self.update_mode("empty")
        else:
            print("Invalid key, please press one of L,D,T,E")

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
