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
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from qt_gui_py_common.worker_thread import WorkerThread
from python_qt_binding.QtCore import Qt, QTimer, Slot
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
        self._publisher2 = rospy.Publisher('/learn_progress', String, queue_size=1)

        # lauras computer -- kinect
        # self._subscriber = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self._callback, queue_size = 1)
        # fuego -- kinect task images
        self._subscriber = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self._callback, queue_size = 1)
        # for object learning fire wire
        self._subscriber2 = rospy.Subscriber('/cam1/camera/image_raw/compressed', CompressedImage, self._callback2, queue_size = 1)
        # for object detection
        self._subscriber3 = rospy.Subscriber('/cam2/camera/image_raw/compressed', CompressedImage, self._callback3, queue_size = 1)
        # for learning progress
        self._subscriber4 = rospy.Subscriber('/learn_progress', String, self._callback4, queue_size = 1)

        # self._bridge = CvBridge()
        # setup kuka widget 
        self._widget = KukaViewWidget()
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        # set to check if path is ready to publish incrementally
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._publish_check)
        self._update_parameter_timer.start(100)

        self._temp_timer = QTimer(self)
        self._temp_timer.timeout.connect(self._publish_progress)
        self._temp_timer.start(1000)

        self.progress = 1

    @Slot(str)
    def _publish_check(self):
        # check if ready to publish
        path = self._widget.check_if_path_ready()
        if path != None:
            self._publisher.publish(str(path))

    @Slot(str)
    def _publish_progress(self):
        self.progress += 1
        self._publisher2.publish(str(self.progress))

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def _shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()

    @Slot(str)
    def _callback(self, ros_data):
        # check for new image frame and display
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.check_task(frame)

    @Slot(str)
    def _callback2(self, ros_data):
        # check for new image frame and display
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.check_learning(frame)

    @Slot(str)
    def _callback3(self, ros_data):
        # check for new image frame and display
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._widget.check_detection(frame)

    @Slot(str)
    def _callback4(self, data):
        self._widget.show_progress(int(data.data))

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
