#!/usr/bin/python3

from flexx import config
from flexxros import node
from flexxros.node import ROSNode
from flexxros.widgets import ROSActionClientWidget
from flexxros.msg import TestAction, TestFeedback, TestResult
import rospy
import actionlib

node_name = "action_example_interface"
config.hostname = "localhost"
config.port = 8097

class ActionServer(object):

    _feedback = TestFeedback()
    _result = TestResult()

    def __init__(self, name):
        self._as = actionlib.SimpleActionServer(name, TestAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        self._feedback.data = "Got " + goal.data + ", started running..."
        self._as.publish_feedback(self._feedback)
        rospy.sleep(1.)
        self._feedback.data = "Almost done..."
        self._as.publish_feedback(self._feedback)
        rospy.sleep(1.)
        self._result.data = "Success with " + goal.data + "!"
        self._as.set_succeeded(self._result)

class ActionExampleInterface(ROSNode):

    def init(self):
        self.client = ROSActionClientWidget(node_name, "flexxros/Test")

rospy.init_node(node_name, anonymous=True)
server = ActionServer(node_name)
node.spin(ActionExampleInterface)
