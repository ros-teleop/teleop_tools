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

from geometry_msgs.msg import Twist
from joy_teleop_testing_common import generate_joy_test_description, TestJoyTeleop

import launch_testing
import pytest
import rclpy


@pytest.mark.launch_test
@launch_testing.parametrize('behavior', ['', 'all', 'any'])
def generate_test_description(behavior):
    parameters = {}
    parameters['twist.type'] = 'topic'
    parameters['twist.interface_type'] = 'geometry_msgs/msg/Twist'
    parameters['twist.topic_name'] = '/cmd_vel'
    parameters['twist.deadman_buttons'] = [2, 4]
    parameters['twist.deadman_behavior'] = behavior
    parameters['twist.axis_mappings.linear-x.axis'] = 1
    parameters['twist.axis_mappings.linear-x.scale'] = 0.8
    parameters['twist.axis_mappings.linear-x.offset'] = 0.0
    parameters['twist.axis_mappings.angular-z.axis'] = 3
    parameters['twist.axis_mappings.angular-z.scale'] = 0.5
    parameters['twist.axis_mappings.angular-z.offset'] = 0.0

    return generate_joy_test_description(parameters)


class MyTestCase(TestJoyTeleop):

    def setUp(self):
        super().setUp()
        self.twist: Twist = None
        self.future = rclpy.task.Future()

        qos = rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                                   depth=1,
                                   reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                                   durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE)

        self.twist_subscriber = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.receive_twist,
            qos,
        )

    def tearDown(self):
        self.node.destroy_subscription(self.twist_subscriber)
        super().tearDown()

    def receive_twist(self, msg):
        self.twist = msg
        self.future.set_result(True)

    def test_all_deadman_button_pressed(self, behavior):
        try:
            # Here we simulate the button 2, 4 to be pressed
            self.joy_msg.buttons = [0, 0, 1, 0, 1]
            self.joy_msg.axes = [0.0, 1.0, 0.0, 1.0]

            self.executor.spin_until_future_complete(self.future, timeout_sec=1)

            self.joy_publisher.publish(self.joy_msg)
            self.node.executor.spin_once(1)

            # Check
            self.assertTrue(self.future.done() and self.future.result(),
                            'Timed out waiting for twist topic to complete')
            self.assertEqual(self.twist.linear.x, 0.8)
            self.assertEqual(self.twist.angular.z, 0.5)

        finally:
            # Cleanup
            self.node.destroy_subscription(self.twist_subscriber)

    def test_some_deadman_button_pressed(self, behavior):

        try:
            # This time only button 2 is pressed and not button 4
            self.joy_msg.buttons = [0, 0, 1, 0, 0]
            self.joy_msg.axes = [0.0, 1.0, 0.0, 1.0]

            self.executor.spin_until_future_complete(self.future, timeout_sec=1)

            self.publish_from_timer = False
            self.joy_publisher.publish(self.joy_msg)
            self.node.executor.spin_once(1)

            # Check
            if behavior in ['any', '']:
                # the default behavior or 'any' is that if any of the deadman button is pressed
                # the message will pass through
                self.assertTrue(self.future.done() and self.future.result(),
                                'Timed out waiting for twist topic to complete')
                self.assertEqual(self.twist.linear.x, 0.8)
                self.assertEqual(self.twist.angular.z, 0.5)
            elif behavior == 'all':
                # the 'all' behavior is that all deadman button must press simultaneously
                # for the message to pass through
                self.assertFalse(self.future.done() and self.future.result(),
                                 'Received a message that instead of none')

        finally:
            # Cleanup
            self.node.destroy_subscription(self.twist_subscriber)
