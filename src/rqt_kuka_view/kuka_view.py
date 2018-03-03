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
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from .area_trajectory import calculate_trajectory

class KukaViewWidget(QWidget):

    def __init__(self):
        super(KukaViewWidget, self).__init__()

        self.setWindowTitle('Task Interface')
        self.window_size = self.size()
        self.send_path = False
        self.calculate_area = False
        self.path = []
        self.counter = 0
        self.capture = None

        self.pen = QPen(Qt.red)
        self.pen.setWidth(8)
        self.pen.setCapStyle(Qt.RoundCap)
        self.pen.setJoinStyle(Qt.RoundJoin)

        self.setup_ui()


    def setup_ui(self):
        self.layout = QVBoxLayout()
        self.button_layout = QHBoxLayout()
        self.image_layout = QHBoxLayout()

        self.video_label = QLabel()

        self.area_button = QPushButton('set area task')
        self.area_button.clicked.connect(self.area_task)

        self.clear_button = QPushButton('clear tasks')
        self.clear_button.clicked.connect(self.clear_path)

        self.move_button = QPushButton('send tasks')
        self.move_button.clicked.connect(self.move_along_path)
       
        self.button_layout.addWidget(self.area_button)
        self.button_layout.addWidget(self.clear_button)
        self.button_layout.addWidget(self.move_button)

        self.image_layout.addWidget(self.video_label)
        self.layout.addLayout(self.button_layout)
        self.layout.addLayout(self.image_layout)

        self.setLayout(self.layout)


    def image_display(self, frame):
        frame = cv2.flip(frame, 1)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaledToWidth(self.window_size.width() - 30)
        if self.path:
            for i in range(len(self.path) - 2):
                if self.path[i] == None or self.path[i + 1] == None:
                    continue
                else:
                    painter = QPainter(pixmap)
                    painter.setPen(self.pen)
                    y_offset = ((self.window_size.height() - pixmap.size().height()) / 2) - 30
                    painter.drawLine(self.path[i][0], self.path[i][1] - y_offset, self.path[i + 1][0], self.path[i + 1][1] - y_offset)
                    painter.end()
        self.video_label.setPixmap(pixmap)
  
    def resizeEvent(self, QResizeEvent):
        self.window_size = self.size()

    def mouseMoveEvent(self, QMouseEvent):
        if not self.send_path:
            pos = self.video_label.mapFromGlobal(QMouseEvent.globalPos())
            if self.calculate_area:
                self.counter += 1
            self.path.append([pos.x(), pos.y()])
    
    def mouseReleaseEvent(self, QMouseEvent):
        if self.calculate_area:
            print self.path
            area = calculate_trajectory(self.path[-self.counter or None:])
            if area == None:
                print 'Please select an area'
                return
            else:
                self.path = self.path[:-self.counter or None]
                for p in area:
                    self.path.append(p)
                self.calculate_area = False
                self.counter = 0
        else:
            self.path.append(None)

    def move_along_path(self):
        self.send_path = True

    def check_if_ready(self):
        if self.send_path:
            return self.path
        else:
            return None
    
    def clear_path(self):
        del self.path[:]
        self.send_path = False

    def area_task(self):
        self.calculate_area = True
        self.send_path = False

    def save_settings(self, plugin_settings, instance_settings):
        # self._nav_view.save_settings(plugin_settings, instance_settings)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # self._nav_view.restore_settings(plugin_settings, instance_settings)
        pass
