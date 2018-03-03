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
from sensor_msgs.msg import Image
from qt_gui_py_common.worker_thread import WorkerThread
from python_qt_binding.QtCore import Qt, QTimer, Slot
from cv_bridge import CvBridge, CvBridgeError
import rospy

class KukaViewPlugin(Plugin):
    def __init__(self, context):
        super(KukaViewPlugin, self).__init__(context)
        if context.serial_number() > 1:
            raise RuntimeError("You may not run more than one instance of kuka_view.")
        self.setObjectName('Task Interface')
        # setup pubisher and subscriber
        self._publisher = rospy.Publisher('trajectory', String, queue_size=100)
        self._subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self._callback, queue_size = 1)
        self._bridge = CvBridge()
        # setup kuka widget 
        self._widget = KukaViewWidget()
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        # set to check if path is ready to publish incrementally
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._publish_check)
        self._update_parameter_timer.start(100)

    @Slot(str)
    def _publish_check(self):
        path = self._widget.check_if_ready()
        if path != None:
            self._publisher.publish(str(path))

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def _shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()

    @Slot(str)
    def _callback(self, data):
        try:
            frame = self._bridge.imgmsg_to_cv2(data, "bgr8")
            self._widget.image_display(frame)
        except CvBridgeError as e:
            print(e)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
