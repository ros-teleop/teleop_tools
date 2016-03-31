#!/usr/bin/env python
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState as JTCS
from teleop_tools_msgs.msg import IncrementAction as TTIA


class IncrementerServer:
    def __init__(self, controller_ns):
        self._as = actionlib.SimpleActionServer("increment",
                                                TTIA,
                                                execute_cb=self._as_cb,
                                                auto_start=False)
        self._command_pub = rospy.Publisher("command",
                                            JointTrajectory,
                                            queue_size=1)
        state = rospy.wait_for_message("state", JTCS)
        self._value = state.actual.positions
        self._goal = JointTrajectory()
        self._goal.joint_names = state.joint_names
        rospy.loginfo('Connected to ' + controller_ns)
        self._as.start()

    def _as_cb(self, goal):
        self.increment_by(goal.increment_by)
        self._as.set_succeeded([])

    def increment_by(self, increment):
        state = rospy.wait_for_message("state", JTCS)
        self._value = state.actual.positions
        self._value = [x + y for x, y in zip(self._value, increment)]
        rospy.loginfo('Sent goal of ' + str(self._value))
        point = JointTrajectoryPoint()
        point.positions = self._value
        point.time_from_start = rospy.Duration(0.1)
        self._goal.points = [point]
        self._command_pub.publish(self._goal)

if __name__ == "__main__":
    rospy.init_node('incrementer_server')
    controller_namespace = 'spine_controller'
    server = IncrementerServer(controller_namespace)
    rospy.spin()
