#!/usr/bin/env python
import importlib

import rospy
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from threading import Thread
from rosservice import ROSServiceException

class JoyTeleopException(Exception):
    pass


class JoyTeleop:
    """
    Generic joystick teleoperation node.
    Will not start without configuration, has to be stored in 'teleop' parameter.
    See config/joy_teleop.yaml for an example.
    """
    def __init__(self):
        if not rospy.has_param("teleop"):
            rospy.logfatal("no configuration was found, taking node down")
            raise JoyTeleopException("no config")

        self.publishers = {}
        self.al_clients = {}
        self.srv_clients = {}
        self.service_types = {}
        self.message_types = {}
        self.command_list = {}
        self.offline_actions = []
        self.offline_services = []

        self.old_buttons = []

        teleop_cfg = rospy.get_param("teleop")

        for i in teleop_cfg:
            if i in self.command_list:
                rospy.logerr("command {} was duplicated".format(i))
                continue
            action_type = teleop_cfg[i]['type']
            self.add_command(i, teleop_cfg[i])
            if action_type == 'topic':
                self.register_topic(i, teleop_cfg[i])
            elif action_type == 'action':
                self.register_action(i, teleop_cfg[i])
            elif action_type == 'service':
                self.register_service(i, teleop_cfg[i])
            else:
                rospy.logerr("unknown type '%s' for command '%s'", action_type, i)

        # Don't subscribe until everything has been initialized.
        rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.joy_callback)

        # Run a low-freq action updater
        rospy.Timer(rospy.Duration(2.0), self.update_actions)

    def joy_callback(self, data):
        try:
            for c in self.command_list:
                if self.match_command(c, data.buttons):
                    self.run_command(c, data)
        except JoyTeleopException as e:
            rospy.logerr("error while parsing joystick input: %s", str(e))
        self.old_buttons = data.buttons

    def register_topic(self, name, command):
        """Add a topic publisher for a joystick command"""
        topic_name = command['topic_name']
        try:
            topic_type = self.get_message_type(command['message_type'])
            self.publishers[topic_name] = rospy.Publisher(topic_name, topic_type, queue_size=1)
        except JoyTeleopException as e:
            rospy.logerr("could not register topic for command {}: {}".format(name, str(e)))

    def register_action(self, name, command):
        """Add an action client for a joystick command"""
        action_name = command['action_name']
        try:
            action_type = self.get_message_type(self.get_action_type(action_name))
            self.al_clients[action_name] = actionlib.SimpleActionClient(action_name, action_type)
            if action_name in self.offline_actions:
                self.offline_actions.remove(action_name)
        except JoyTeleopException:
            if action_name not in self.offline_actions:
                self.offline_actions.append(action_name)

    class AsyncServiceProxy(object):
        def __init__(self, name, service_class, persistent):
            try:
                rospy.wait_for_service(name, timeout=2.0)
            except ROSException:
                raise JoyTeleopException("Service {} is not available".format(name))
            self._service_proxy = rospy.ServiceProxy(name, service_class, persistent)
            self._thread = Thread(target=self._service_proxy, args=[])

        def __del__(self):
            # try to join our thread - no way I know of to interrupt a service
            # request
            if self._thread.is_alive():
                self._thread.join(1.0)

        def __call__(self, request):
            if self._thread.is_alive():
                self._thread.join(0.01)
                if self._thread.is_alive():
                    return False

            self._thread = Thread(target=self._service_proxy, args=[request])
            self._thread.start()
            return True

    def register_service(self, name, command):
        """ Add an AsyncServiceProxy for a joystick command """
        service_name = command['service_name']
        if 'service_persistent' not in command:
            command['service_persistent'] = False
        service_persistent = command['service_persistent']
        try:
            service_type = self.get_service_type(service_name)
            self.srv_clients[service_name] = self.AsyncServiceProxy(
                service_name,
                service_type,
                service_persistent)

            if service_name in self.offline_services:
                self.offline_services.remove(service_name)
        except JoyTeleopException:
            if service_name not in self.offline_services:
                self.offline_services.append(service_name)

    def match_command(self, c, buttons):
        """Find a command matching a joystick configuration"""
        for b in self.command_list[c]['buttons']:
            if b < 0 or len(buttons) <= b or buttons[b] != 1:
                return False
        return sum(buttons) >= len(self.command_list[c]['buttons'])

    def add_command(self, name, command):
        """Add a command to the command list"""
        if command['type'] == 'topic':
            if 'deadman_buttons' not in command:
                command['deadman_buttons'] = []
            command['buttons'] = command['deadman_buttons']
        elif command['type'] == 'action':
            if 'action_goal' not in command:
                command['action_goal'] = {}
        elif command['type'] == 'service':
            if 'service_request' not in command:
                command['service_request'] = {}
        self.command_list[name] = command

    def run_command(self, command, joy_state):
        """Run a joystick command"""
        cmd = self.command_list[command]
        if cmd['type'] == 'topic':
            self.run_topic(command, joy_state)
        elif cmd['type'] == 'action':
            if cmd['action_name'] in self.offline_actions:
                rospy.logerr("command {} was not played because the action "
                             "server was unavailable. Trying to reconnect..."
                             .format(cmd['action_name']))
                self.register_action(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_action(command, joy_state)
        elif cmd['type'] == 'service':
            if cmd['service_name'] in self.offline_services:
                rospy.logerr("command {} was not played because the service "
                             "server was unavailable. Trying to reconnect..."
                             .format(cmd['service_name']))
                self.register_service(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_service(command, joy_state)
        else:
            raise JoyTeleopException('command {} is neither a topic publisher nor an action or service client'
                                     .format(command))

    def run_topic(self, c, joy_state):
        cmd = self.command_list[c]
        msg = self.get_message_type(cmd['message_type'])()

        if 'message_value' in cmd:
            for param in cmd['message_value']:
                self.set_member(msg, param['target'], param['value'])

        else:
            for mapping in cmd['axis_mappings']:
                if 'axis' in mapping:
                    if len(joy_state.axes) > mapping['axis']:
                        val = joy_state.axes[mapping['axis']] * mapping.get('scale', 1.0) + mapping.get('offset', 0.0)
                    else:
                        rospy.logerr('Joystick has only {} axes (indexed from 0), but #{} was referenced in config.'.format(len(joy_state.axes), mapping['axis']))
                        val = 0.0
                elif 'button' in mapping:
                    if len(joy_state.buttons) > mapping['button']:
                        val = joy_state.buttons[mapping['button']] * mapping.get('scale', 1.0) + mapping.get('offset', 0.0)
                    else:
                        rospy.logerr('Joystick has only {} buttons (indexed from 0), but #{} was referenced in config.'.format(len(joy_state.buttons), mapping['button']))
                        val = 0.0
                else:
                    rospy.logerr('No Supported axis_mappings type found in: {}'.format(mapping))
                    val = 0.0

                self.set_member(msg, mapping['target'], val)

        # If there is a stamp field, fill it with rospy.Time.now()
        if hasattr(msg, 'header'):
            msg.header.stamp = rospy.Time.now()

        self.publishers[cmd['topic_name']].publish(msg)

    def run_action(self, c, joy_state):
        cmd = self.command_list[c]
        goal = self.get_message_type(self.get_action_type(cmd['action_name'])[:-6] + 'Goal')()
        genpy.message.fill_message_args(goal, [cmd['action_goal']])
        self.al_clients[cmd['action_name']].send_goal(goal)

    def run_service(self, c, joy_state):
        cmd = self.command_list[c]
        request = self.get_service_type(cmd['service_name'])._request_class()
        # should work for requests, too
        genpy.message.fill_message_args(request, [cmd['service_request']])
        if not self.srv_clients[cmd['service_name']](request):
            rospy.loginfo('Not sending new service request for command {} because previous request has not finished'
                          .format(c))

    def set_member(self, msg, member, value):
        ml = member.split('.')
        if len(ml) < 1:
            return
        target = msg
        for i in ml[:-1]:
            target = getattr(target, i)
        setattr(target, ml[-1], value)

    def get_message_type(self, type_name):
        if type_name not in self.message_types:
            try:
                package, message = type_name.split('/')
                mod = importlib.import_module(package + '.msg')
                self.message_types[type_name] = getattr(mod, message)
            except ValueError:
                raise JoyTeleopException("message type format error")
            except ImportError:
                raise JoyTeleopException("module {} could not be loaded".format(package))
            except AttributeError:
                raise JoyTeleopException("message {} could not be loaded from module {}".format(package, message))
        return self.message_types[type_name]

    def get_action_type(self, action_name):
        try:
            return rostopic._get_topic_type(rospy.resolve_name(action_name) + '/goal')[0][:-4]
        except TypeError:
            raise JoyTeleopException("could not find action {}".format(action_name))

    def get_service_type(self, service_name):
        if service_name not in self.service_types:
            try:
                self.service_types[service_name] = rosservice.get_service_class_by_name(service_name)
            except ROSServiceException as e:
                raise JoyTeleopException("service {} could not be loaded: {}".format(service_name, str(e)))
        return self.service_types[service_name]

    def update_actions(self, evt=None):
        for name, cmd in self.command_list.items():
            if cmd['type'] != 'action':
                continue
            if cmd['action_name'] in self.offline_actions:
                self.register_action(name, cmd)


if __name__ == "__main__":
    try:
        rospy.init_node('joy_teleop')
        jt = JoyTeleop()
        rospy.spin()
    except JoyTeleopException:
        pass
    except rospy.ROSInterruptException:
        pass
