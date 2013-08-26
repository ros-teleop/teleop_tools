#!/usr/bin/env python
import roslib; roslib.load_manifest('joy_teleop')

import importlib

import rospy
import sensor_msgs.msg
import actionlib
import rostopic

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

        teleop_cfg = rospy.get_param("teleop")
        rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.joy_callback)

        self.publishers = {}
        self.al_clients = {}
        self.message_types = {}
        self.command_list = {}
        self.offline_actions = []

        self.old_buttons = []

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
            else:
                rospy.logerr("unknown type '%s' for command '%s'", action_type, i)

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
            self.publishers[topic_name] = rospy.Publisher(topic_name, topic_type)
        except JoyTeleopException as e:
            rospy.logerr("could not regiter topic for command {}: {}".format(name, str(e)))

    def register_action(self, name, command):
        """Add an action client for a joystick command"""
        action_name = command['action_name']
        try:
            action_type = self.get_message_type(self.get_action_type(action_name))
            self.al_clients[action_name] = actionlib.SimpleActionClient(action_name, action_type)
            if action_name in self.offline_actions:
                self.offline_actions.remove(action_name)
        except JoyTeleopException as e:
            rospy.logerr("could not register action for command {}: {}".format(name, str(e)))
            if action_name not in self.offline_actions:
                self.offline_actions.append(action_name)

    def match_command(self, c, buttons):
        """Find a command mathing a joystick configuration"""
        for b in self.command_list[c]['buttons']:
            if buttons[b] != 1:
                return False
        return sum(buttons) == len(self.command_list[c]['buttons'])

    def add_command(self, name, command):
        """Add a command to the command list"""
        if command['type'] == 'topic':
            command['buttons']= command['deadman_buttons']
        self.command_list[name] = command

    def run_command(self, command, joy_state):
        """Run a joystick command"""
        cmd = self.command_list[command]
        if cmd['type'] == 'topic':
            self.run_topic(command, joy_state)
        elif cmd['type']== 'action':
            if cmd['action_name'] in self.offline_actions:
                self.register_action(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_action(command, joy_state)
        else:
            raise JoyTeleopException('command {} is neither a topic publisher nor an action client'
                                     .format(command))

    def run_topic(self, c, joy_state):
        cmd = self.command_list[c]
        msg = self.get_message_type(cmd['message_type'])()
        for mapping in cmd['axis_mappings']:
            val = joy_state.axes[mapping['axis']] * mapping.get('scale', 1.0) + mapping.get('offset', 0.0)
            self.set_member(msg, mapping['target'], val)
        self.publishers[cmd['topic_name']].publish(msg)

    def run_action(self, c, joy_state):
        cmd = self.command_list[c]
        goal = self.get_message_type(self.get_action_type(cmd['action_name'])[:-6] + 'Goal')()
        self.fill_msg(goal, cmd['action_goal'])
        self.al_clients[cmd['action_name']].send_goal(goal)

    def fill_msg(self, msg, values):
        for k, v in values.iteritems():
            if type(v) is dict:
                self.fill_msg(getattr(msg, k), v)
            else:
                setattr(msg, k, v)

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


if __name__ == "__main__":
    try:
        rospy.init_node('joy_teleop')
        jt = JoyTeleop()
        rospy.spin()
    except JoyTeleopException:
        pass
    except rospy.ROSInterruptException:
        pass
