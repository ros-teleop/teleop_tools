# -*- coding: utf-8 -*-
#
# Copyright (c) 2020 Open Source Robotics Foundation
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of the copyright holder nor the names of its
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

from joy_teleop_testing_common import generate_joy_test_description, TestJoyTeleop
import pytest
import rclpy
from std_msgs.msg import UInt8MultiArray


@pytest.mark.rostest
def generate_test_description():
    parameters = {}
    parameters['array3.type'] = 'topic'
    parameters['array3.interface_type'] = 'std_msgs/msg/UInt8MultiArray'
    parameters['array3.topic_name'] = '/array3'
    parameters['array3.deadman_buttons'] = [0]

    parameters['array3.axis_mappings.data.index'] = 0
    parameters['array3.axis_mappings.data.axis'] = 0
    parameters['array3.axis_mappings.data.scale'] = 1
    parameters['array3.axis_mappings.data.offset'] = 0

    parameters['array3.axis_mappings.data-.index'] = 1
    parameters['array3.axis_mappings.data-.axis'] = 1
    parameters['array3.axis_mappings.data-.scale'] = 1
    parameters['array3.axis_mappings.data-.offset'] = 0

    parameters['array3.axis_mappings.data--.index'] = 4
    parameters['array3.axis_mappings.data--.axis'] = 2
    parameters['array3.axis_mappings.data--.scale'] = 1
    parameters['array3.axis_mappings.data--.offset'] = 0

    return generate_joy_test_description(parameters)


class ArrayIndexingMappingTestSuite(TestJoyTeleop):

    def publish_message(self):
        self.joy_publisher.publish(self.joy_msg)

    def test_array_mapping(self):
        array: UInt8MultiArray = None
        future = rclpy.task.Future()

        def receive_array(msg):
            nonlocal array
            nonlocal future

            array = msg
            future.set_result(True)

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        array_subscriber = self.node.create_subscription(
            UInt8MultiArray,
            '/array3',
            receive_array,
            qos,
        )

        try:
            self.joy_msg.buttons = [1]  # deadman button pressed
            self.joy_msg.axes = [1.0, 1.0, 1.0]

            self.executor.spin_until_future_complete(future, timeout_sec=10)

            # Check
            self.assertTrue(future.done() and future.result(),
                            'Timed out waiting for array topic to complete')
            self.assertSequenceEqual(array.data, [1, 1, 0, 0, 1])

        finally:
            # Cleanup
            self.node.destroy_subscription(array_subscriber)


if __name__ == '__main__':
    pytest.main()
