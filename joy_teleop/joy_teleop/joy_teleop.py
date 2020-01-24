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

import importlib

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from rosidl_runtime_py import set_message_fields
import sensor_msgs.msg


class JoyTeleopException(Exception):
    pass


class JoyTeleop(Node):
    """
    Generic joystick teleoperation node.

    Will not start without configuration, has to be stored in 'teleop' parameter.
    See config/joy_teleop.yaml for an example.
    """

    def __init__(self):
        super().__init__('joy_teleop', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.config = {}
        self.pubs = {}
        self.al_clients = {}
        self.srv_clients = {}
        self.message_types = {}
        self.command_list = {}
        self.offline_actions = []
        self.offline_services = []

        self.old_buttons = []

        self.retrieve_config()

        for i, config in self.config.items():
            if i in self.command_list:
                self.get_logger().error('command {} was duplicated'.format(i))
                continue

            try:
                interface_group = config['type']

                self.add_command(i, config)

                if interface_group == 'topic':
                    self.register_topic(i, config)
                elif interface_group == 'action':
                    self.register_action(i, config)
                elif interface_group == 'service':
                    self.register_service(i, config)
                else:
                    self.get_logger().error("unknown type '{type}'"
                                            "for command '{i}'".format_map(locals()))
            except TypeError:
                self.get_logger().warn(f"parameter {i} is not a dict")

        # Don't subscribe until everything has been initialized.
        self._subscription = self.create_subscription(
            sensor_msgs.msg.Joy, 'joy', self.joy_callback, 1)

        # Run a low-freq action updater
        self._timer = self.create_timer(2.0, self.update_actions)

    def retrieve_config(self):
        for param_name in sorted(list(self._parameters.keys())):
            pval = self.get_parameter(param_name).value
            self.insert_dict(self.config, param_name, pval)

    def insert_dict(self, dictionary, key, value):
        split = key.split(PARAMETER_SEPARATOR_STRING, 1)
        if len(split) > 1:
            if not split[0] in dictionary:
                dictionary[split[0]] = {}
            self.insert_dict(dictionary[split[0]], split[1], value)
        else:
            dictionary[key] = value

    def joy_callback(self, data):
        try:
            for c in self.command_list:
                if self.match_button_command(c, data.buttons) or self.match_axis_command(c, data.axes):
                    self.run_command(c, data)
        except JoyTeleopException as e:
            self.get_logger().error('error while parsing joystick input: %s', str(e))
        self.old_buttons = data.buttons

    def register_topic(self, name, command):
        """Add a topic publisher for a joystick command."""
        topic_name = command['topic_name']
        try:
            topic_type = self.get_interface_type(command['interface_type'], '.msg')
            self.pubs[topic_name] = self.create_publisher(topic_type, topic_name, 1)
        except JoyTeleopException as e:
            self.get_logger().error(
                'could not register topic for command {}: {}'.format(name, str(e)))

    def register_action(self, name, command):
        """Add an action client for a joystick command."""
        action_name = command['action_name']
        if action_name not in self.al_clients:
            action_type = self.get_interface_type(command['interface_type'], '.action')
            self.al_clients[action_name] = ActionClient(self, action_type, action_name)

        if self.al_clients[action_name].server_is_ready():
            if action_name in self.offline_actions:
                self.offline_actions.remove(action_name)
        else:
            if action_name not in self.offline_actions:
                self.get_logger().warn(
                    'action {} is not read yet'.format(action_name))
                self.offline_actions.append(action_name)

    class AsyncServiceProxy(object):

        def __init__(self, node, service_name, service_type):
            self._service_client = node.create_client(service_type, service_name)
            if not self._service_client.wait_for_service(timeout_sec=1.0):
                raise JoyTeleopException('Service {} is not available'.format(service_name))

        def __call__(self, request):
            self._service_client.call_async(request)
            return True

    def register_service(self, name, command):
        """Add an AsyncServiceProxy for a joystick command."""
        service_name = command['service_name']
        try:
            service_type = self.get_interface_type(command['interface_type'], '.srv')
            self.srv_clients[service_name] = self.AsyncServiceProxy(
                self,
                service_name,
                service_type)

            if service_name in self.offline_services:
                self.offline_services.remove(service_name)
        except JoyTeleopException:
            if service_name not in self.offline_services:
                self.offline_services.append(service_name)

    def match_button_command(self, c, buttons):
        """Find a command matching a joystick configuration."""
        if len(buttons) == 0 or not 'buttons' in self.command_list[c] or len(self.command_list[c]['buttons']) == 0 or \
           len(buttons) <= max(self.command_list[c]['buttons']):
            return False
        return any(buttons[cmd_button] for cmd_button in self.command_list[c]['buttons'])

    def match_axis_command(self, c, axes):
        """Find a command matching a joystick configuration."""
        if len(axes) == 0 or not 'axes' in self.command_list[c] or len(self.command_list[c]['axes']) == 0 or \
           len(axes) <= max(int(cmd_axis) for cmd_axis, value in self.command_list[c]['axes'].items()):
            return False

        # We know there are enough axes that we *might* match.  Iterate through
        # each of the incoming axes, and if they are totally pressed (1.0),
        # we have a match and should run the command.
        for cmd_axis, value in self.command_list[c]['axes'].items():
            if axes[int(cmd_axis)] == value:
                return True

        return False

    def add_command(self, name, command):
        """Add a command to the command list."""
        if command['type'] == 'topic':
            if 'deadman_buttons' not in command:
                command['deadman_buttons'] = []
            command['buttons'] = command['deadman_buttons']
            if 'deadman_axes' not in command:
                command['deadman_axes'] = []
            command['axes'] = command['deadman_axes']
        elif command['type'] == 'action':
            if 'action_goal' not in command:
                command['action_goal'] = {}
        elif command['type'] == 'service':
            if 'service_request' not in command:
                command['service_request'] = {}
        self.command_list[name] = command

    def run_command(self, command, joy_state):
        """Run a joystick command."""
        cmd = self.command_list[command]
        if cmd['type'] == 'topic':
            self.run_topic(command, joy_state)
        elif cmd['type'] == 'action':
            if cmd['action_name'] in self.offline_actions:
                self.get_logger().error('command {} was not played because the action '
                                        'server was unavailable. Trying to reconnect...'
                                        .format(cmd['action_name']))
                self.register_action(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_action(command, joy_state)
        elif cmd['type'] == 'service':
            if cmd['service_name'] in self.offline_services:
                self.get_logger().error('command {} was not played because the service '
                                        'server was unavailable. Trying to reconnect...'
                                        .format(cmd['service_name']))
                self.register_service(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_service(command, joy_state)
        else:
            raise JoyTeleopException(
                'command {} is neither a topic publisher nor an action or service client'
                .format(command))

    def run_topic(self, c, joy_state):
        cmd = self.command_list[c]
        msg = self.get_interface_type(cmd['interface_type'], '.msg')()

        if 'message_value' in cmd:
            if cmd['message_value'] is not None:
                for target, param in cmd['message_value'].items():
                    self.set_member(msg, target, param['value'])

        else:
            for mapping, values in cmd['axis_mappings'].items():
                if 'axis' in values:
                    if len(joy_state.axes) > values['axis']:
                        val = joy_state.axes[values['axis']] * values.get('scale', 1.0) + \
                            values.get('offset', 0.0)
                    else:
                        self.get_logger().error('Joystick has only {} axes (indexed from 0),'
                                                'but #{} was referenced in config.'.format(
                                                    len(joy_state.axes), values['axis']))
                        val = 0.0
                elif 'button' in values:
                    if len(joy_state.buttons) > values['button']:
                        val = joy_state.buttons[values['button']] * values.get('scale', 1.0) + \
                            values.get('offset', 0.0)
                    else:
                        self.get_logger().error('Joystick has only {} buttons (indexed from 0),'
                                                'but #{} was referenced in config.'.format(
                                                    len(joy_state.buttons), values['button']))
                        val = 0.0
                else:
                    self.get_logger().error(
                        'No Supported axis_mappings type found in: {}'.format(mapping))
                    val = 0.0

                self.set_member(msg, mapping, val)

        # If there is a stamp field, fill it with rospy.Time.now()
        if hasattr(msg, 'header'):
            msg.header.stamp = self.get_clock().now()

        self.pubs[cmd['topic_name']].publish(msg)

    def run_action(self, c, joy_state):
        cmd = self.command_list[c]
        goal = self.get_interface_type(cmd['interface_type'], '.action').Goal()
        for target, value in cmd['action_goal'].items():
            set_message_fields(goal, {target: value})

        # No need to wait
        self.al_clients[cmd['action_name']].send_goal_async(goal)

    def run_service(self, c, joy_state):
        cmd = self.command_list[c]
        request = self.get_interface_type(cmd['interface_type'], '.srv').Request()
        for target, value in cmd['service_request'].items():
            set_message_fields(request, {target: value})
        if not self.srv_clients[cmd['service_name']](request):
            self.get_logger().info('Not sending new service request for command {} '
                                   'because previous request has not finished'.format(c))

    def set_member(self, msg, member, value):
        ml = member.split('-')
        if len(ml) < 1:
            return
        target = msg
        for i in ml[:-1]:
            target = getattr(target, i)
        setattr(target, ml[-1], value)

    def get_interface_type(self, type_name, ext):
        if type_name not in self.message_types:
            try:
                package, interface, message = type_name.split('/')
                mod = importlib.import_module(package + ext)
                self.message_types[type_name] = getattr(mod, message)
            except ValueError:
                raise JoyTeleopException('message type format error')
            except ImportError:
                raise JoyTeleopException('module {} could not be loaded'.format(package))
            except AttributeError:
                raise JoyTeleopException(
                    'message {} could not be loaded from module {}'.format(package, message))
        return self.message_types[type_name]

    def update_actions(self, evt=None):
        for name, cmd in self.command_list.items():
            if cmd['type'] != 'action':
                continue
            if cmd['action_name'] in self.offline_actions:
                self.register_action(name, cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()

    try:
        rclpy.spin(node)
    except JoyTeleopException as e:
        node.get_logger().error(e.message)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
