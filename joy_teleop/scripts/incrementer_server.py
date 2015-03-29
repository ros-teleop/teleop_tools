#!/usr/bin/env python
import rospy
import actionlib
import trajectory_msgs.msg as TJM
from control_msgs.srv import QueryTrajectoryState as QTS
import teleop_tools_msgs.msg as JTA

class IncrementerServer:
    def __init__(self, controller_ns):
        self._as = actionlib.SimpleActionServer("increment",
            JTA.IncrementAction, execute_cb=self._as_cb, auto_start=False)
        self._command_pub = rospy.Publisher("command", TJM.JointTrajectory, queue_size=2)
        #TODO add timeout
        self._service_client = rospy.ServiceProxy("query_state", QTS, True)
        rospy.sleep(2.0)
        self._service_client.wait_for_service()
        result = self._service_client.call(rospy.Time.now())
        [self._value] = result.position
        self._goal = TJM.JointTrajectory()
        self._goal.joint_names = result.name
        rospy.loginfo('Connected to ' + controller_ns)
        self._as.start()

    def _as_cb(self, goal):
        self.increment_by(goal.increment_by)
        self._as.set_succeeded([])

    def increment_by(self, increment):
        #cancel all goals sent to server
        #TODO validate these values before sending to avoid increasing any well above the limit
        result = self._service_client.call(rospy.Time.now())
        [self._value] = result.position
        self._value += increment
        rospy.loginfo('Sent goal of ' + str(self._value))
        point = TJM.JointTrajectoryPoint()
        point.positions = [self._value]
        point.time_from_start = rospy.Duration(0.1)
        self._goal.points = [point]
        self._command_pub.publish(self._goal)

if __name__ == "__main__":
    rospy.init_node('incrementer_server')
    controller_namespace = 'spine_controller'
    server = IncrementerServer(controller_namespace)
    rospy.spin()
