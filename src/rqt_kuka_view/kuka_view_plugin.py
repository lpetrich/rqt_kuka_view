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
# Initialize plugin widget to run task interface
# Prepared for the Kuka Innovation Award 2018
# One camera must be publishing to the topic /usb_cam/image_raw before running this node
# Publishes to the topic /trajectory
#######################################################################

from .kuka_view import KukaViewWidget
from qt_gui.plugin import Plugin
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage, Image
from qt_gui_py_common.worker_thread import WorkerThread
from python_qt_binding.QtCore import Qt, QTimer
#
from python_qt_binding.QtGui import *
#
from cv_bridge import CvBridge, CvBridgeError
import rospy, cv2
import numpy as np

class KukaViewPlugin(Plugin):
    def __init__(self, context):
        super(KukaViewPlugin, self).__init__(context)
        if context.serial_number() > 1:
            raise RuntimeError("You may not run more than one instance of kuka_view.")
        self.setObjectName('Task Interface')
        # setup pubisher and subscriber
        self._publisher = rospy.Publisher('/chris/targetPoints_2D', String, queue_size=1000)
        # self._publisher2 = rospy.Publisher('/learn_progress', String, queue_size=1)

        # task images
        self._subscriber = rospy.Subscriber('/camera/rgb/image_rect_color/compressed', CompressedImage, self._callback, queue_size = 1)
        # for object learning fire wire
        self._subscriber2 = rospy.Subscriber('/cam1/camera/image_raw/compressed',  CompressedImage, self._callback2, queue_size = 1)
        # for object detection
        self._subscriber3 = rospy.Subscriber('/kinect2/hd/image_color_rect/compressed', CompressedImage, self._callback3, queue_size = 1)
        # subscriber for cropped images
        self._subscriber5 = rospy.Subscriber('/crop', Image, self._callback5)
        self._subscriber6 = rospy.Subscriber('/map', Image, self._callback6,  queue_size = 1)
        self._subscriber7 = rospy.Subscriber('/cl_image', Image, self._callback7,  queue_size = 1)
        self._subscriber8 = rospy.Subscriber('/imgnet_image', Image, self._callback8,  queue_size = 1)

        # for starting/ending the learning process
        self._pub_learn = rospy.Publisher('/command_learn', String, queue_size=1000)
        # for starting/ending the detection process
        self._pub_det = rospy.Publisher('/command_det', String, queue_size=1000)
        # for starting/ending the task process
        self._pub_task = rospy.Publisher('/command_task', String, queue_size=1000)


        # for initiating different modules
        self._pub_init = rospy.Publisher('/command_init', String, queue_size=1000)


        # for learning progress
        # self._subscriber4 = rospy.Subscriber('/learn_progress', String, self._callback4, queue_size = 1)

        self._subscriber_learn_progress = rospy.Subscriber('/learning_progress', String, self._callback_progress, queue_size = 1)

        self._subscriber_feat_progress = rospy.Subscriber('/feature_progress', String, self._callback_feat_progress, queue_size = 1)

        # opencv stuff
        self._bridge = CvBridge()

        # setup kuka widget
        #dw = QDesktopWidget()
        #x = dw.width()*.8
        #y = dw.height()*.8
        #self._widget = KukaViewWidget(self._pub_learn, self._pub_init)
        self._widget = KukaViewWidget()
        #screenSize = QDesktopWidget().availableGeometry()
        screenSize = self._widget.geometry()
        x = screenSize.width()
        y = screenSize.height()
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number())) # set the title
        # This part is for adjusting the window size
        #self._widget.setFixedSize(x,y) # change the default size
        #self._widget.setWindowFlags(Qt.FramelessWindowHint) # hide title bar
        self._widget.setPublisherLearn(self._pub_learn)
        self._widget.setPublisherInit(self._pub_init)
        self._widget.setPublisherDet(self._pub_det)
        self._widget.setPublisherTask(self._pub_task)

        context.add_widget(self._widget)

        # set to check if path is ready to publish incrementally
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._publish_check)
        self._update_parameter_timer.start(100)

        # self._temp_timer = QTimer(self)
        # self._temp_timer.timeout.connect(self._publish_progress)
        # self._temp_timer.start(1000)

        self.progress = 1
        self._widget.init_modules()

    def _publish_check(self):
        # check if ready to publish
        path = self._widget.check_if_path_ready()
        if path != None:
            self._publisher.publish(str(path))

    # def _publish_progress(self):
    #     self.progress += 1
    #     self._publisher2.publish(str(self.progress))

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def _shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()

    def _callback(self, ros_data):
        # check for new image frame and display
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # print frame.shape
        self._widget.check_task(frame)

    def _callback2(self, ros_data):
        # check for new image frame and display
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.check_learning(frame)

    def _callback3(self, ros_data):
        # check for new image frame and display
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.check_detection(frame)

    def _callback5(self, ros_data):
        # check for new image frame and display
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = self._bridge.imgmsg_to_cv2(ros_data, "bgr8")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.show_crop(frame)

    def _callback6(self, ros_data):
        # check for new image frame and display
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = self._bridge.imgmsg_to_cv2(ros_data, "bgr8")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.show_heatmap(frame)

    def _callback7(self, ros_data):
        # check for new image frame and display
        frame = self._bridge.imgmsg_to_cv2(ros_data, "bgr8")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.show_det_after(frame)

    def _callback8(self, ros_data):
        # check for new image frame and display
        frame = self._bridge.imgmsg_to_cv2(ros_data, "bgr8")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.show_det_before(frame)


    def _callback_progress(self, ros_data):
        self._widget.show_progress(int(ros_data.data))

    def _callback_feat_progress(self, ros_data):
        self._widget.show_feat_progress(ros_data.data)


    def _callback4(self, data):
        self._widget.show_progress(int(data.data))

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
