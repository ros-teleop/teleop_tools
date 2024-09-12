#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2019 PAL Robotics SL.
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
#  * Neither the name of PAL Robotics SL. nor the names of its
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
#
#
# Authors:
#   * Bence Magyar
#   * Sam Pfeiffer
#   * Jeremie Deray (artivis)
#   * Borong Yuan

import time

from control_msgs.msg import JointTrajectoryControllerState as JTCS
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from teleop_tools_msgs.action import Increment as TTIA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class IncrementerServer(Node):

    def __init__(self):
        super().__init__('incrementer_server', namespace='joint_trajectory_controller')

        cb_group = ReentrantCallbackGroup()

        self._has_new_message = False

        self._as = ActionServer(self, TTIA, 'increment',
                                self._as_cb, callback_group=cb_group)

        self._command_pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory', 1)

        self._state_sub = self.create_subscription(
            JTCS, 'controller_state', self._state_cb, 1, callback_group=cb_group)

        self._goal = JointTrajectory()
        self.get_logger().info('Connected to {}'.format(self.get_namespace()))

    def _as_cb(self, goal):
        self.increment_by(goal.request.increment_by)
        goal.succeed()
        return TTIA.Result()

    def _state_cb(self, state):
        self._state = state
        self._has_new_message = True

    # TODO(artivis) change after
    # https://github.com/ros2/rclcpp/issues/520
    # has landed
    def _wait_for_new_message(self):
        self._has_new_message = False
        while not self._has_new_message:
            time.sleep(0.01)
        return self._state

    def increment_by(self, increment):
        state = self._wait_for_new_message()
        self._goal.joint_names = state.joint_names
        self._value = state.feedback.positions
        self._value = [x + y for x, y in zip(self._value, increment)]
        self.get_logger().info('Sent goal of {}'.format(self._value))
        point = JointTrajectoryPoint()
        point.positions = self._value
        point.time_from_start = Duration(seconds=0.1).to_msg()
        self._goal.points = [point]
        self._command_pub.publish(self._goal)


def main():
    rclpy.init()

    node = IncrementerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
