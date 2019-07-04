# Copyright 2019 Canonical, Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time
import unittest

import launch
import launch_ros
import launch_ros.actions
# import launch_testing.util
# import launch_testing_ros
import rclpy
import rclpy.context
import rclpy.executors
import sensor_msgs.msg
import std_msgs.msg

from ament_index_python.packages import get_package_share_directory


def generate_test_description(ready_fn):
    # Necessary to get real-time stdout from python processes:
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    parameters_file = os.path.join(
        get_package_share_directory('joy_teleop'),
        'config', 'joy_teleop_example.yaml'
    )

    joy_teleop_node = launch_ros.actions.Node(
        package='joy_teleop',
        node_executable='joy_teleop',
        parameters=[parameters_file],
        env=proc_env,
    )

    # fibonacci_server = launch_ros.actions.Node(
    #     package='action_tutorials',
    #     node_executable='fibonacci_action_server.py'
    # )
    #
    # add_two_ints_server = launch_ros.actions.Node(
    #     package='demo_nodes_cpp',
    #     node_executable='add_two_ints_server'
    # )

    return launch.LaunchDescription([
            joy_teleop_node,
            fibonacci_server,
            add_two_ints_server,
            # Start tests right away - no need to wait for anything
            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
           ]
           # ,
           # {
           #  'joy_teleop': joy_teleop_node
           # }
           )


class TestJoyTeleop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('test_node', context=cls.context)

        # The demo node listener has no synchronization to indicate when it's ready to start
        # receiving messages on the /chatter topic.  This plumb_listener method will attempt
        # to publish for a few seconds until it sees output
        cls.publisher = cls.node.create_publisher(
            sensor_msgs.msg.Joy,
            'joy',
            1
        )
        cls.msg = sensor_msgs.msg.Joy()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()

    def spin_rclpy(self, timeout_sec):
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        executor.add_node(self.node)
        try:
            executor.spin_once(timeout_sec=timeout_sec)
        finally:
            executor.remove_node(self.node)

    def test_topic_hello(self):

        # Expect the talker to publish strings on '/chatter'
        msgs_rx = []
        sub = self.node.create_subscription(
            std_msgs.msg.String,
            'chatter',
            lambda msg: msgs_rx.append(msg),
            10
        )
        self.addCleanup(self.node.destroy_subscription, sub)

        self.msg.buttons = [2]
        self.publisher.publish(self.msg)

        # Wait until the talker transmits messages over the ROS topic
        end_time = time.time() + 10
        while time.time() < end_time:
            self.publisher.publish(self.msg)
            self.spin_rclpy(1.0)
            if len(msgs_rx) > 2:
                break

        self.assertGreater(len(msgs_rx), 1)

        for msg in msgs_rx:
            print(msg)
            assert msg.data == 'Hello'

    # # TODO(artivis)
    # def test_fibonacci(self):
    #
    #     self.msg.buttons = [1]
    #     self.publisher.publish(self.msg)
    #
    #     # Wait until the talker transmits messages over the ROS topic
    #     end_time = time.time() + 10
    #     while time.time() < end_time:
    #         self.publisher.publish(self.msg)
    #         self.spin_rclpy(1.0)
    #
    # # TODO(artivis)
    # def test_add_two_ints(self):
    #
    #     self.msg.buttons = [10]
    #     self.publisher.publish(self.msg)
    #
    #     # Wait until the talker transmits messages over the ROS topic
    #     end_time = time.time() + 10
    #     while time.time() < end_time:
    #         self.publisher.publish(self.msg)
    #         self.spin_rclpy(1.0)
