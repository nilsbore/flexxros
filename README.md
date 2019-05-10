# flexxros
ROS webpage GUIs through [flexx](https://flexx.readthedocs.io/en/stable/).
Full documentation at https://nilsbore.github.io/flexxros-docs/ .

flexxros aims to make it dead easy to create small web interfaces for ROS using only python.

## Dependencies

```
pip3 install flexx rospkg
pip3 install tornado==5
```

## Running

```
rosrun flexxros example.py
```
This will present a webpage on `http://localhost:8097/`.

## Example application

The following script demonstrates a simple publish-subscribe application using flexxros.
It is identical to `example.py` in the `scripts` folder.

```python
from flexx import flx, config
from flexxros import node
from flexxros.node import ROSNode, ROSWidget

class PublishSubscribeWidget(ROSWidget):

    def callback(self, msg):
        self.receive_message.set_text(msg.data)

    @flx.reaction("send_message.pointer_click")
    def _send_message(self, *events):
        self.publish("/test_topic", {"data": self.type_message.text})

    def init(self):
        with flx.FormLayout(flex=1, maxsize=400):
            self.type_message = flx.LineEdit(title="Message to send:", text="")
            self.send_message = flx.Button(text="Publish message")
            self.receive_message = flx.Label(title="Received message:", text="Waiting for message...")

        self.announce_publish("/test_topic", "std_msgs/String")
        self.subscribe("/test_topic", "std_msgs/String", self.callback)

class ExampleInterface(ROSNode):

    def init(self):
        self.main_widget = PublishSubscribeWidget()

config.hostname = "localhost"
config.port = 8097
node.init_and_spin("example_interface", ExampleInterface)
```

## Action service example

The action service example is the same as in `scripts/action_example.py`.
It presents an interface similar to the default `axclient.py` version.

```python
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
```
## Service example

The action service example is the same as in `scripts/service_example.py`.

```python
from flexx import flx, config
from flexxros import node
from flexxros.node import ROSNode, ROSWidget
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

node_name = "action_example_interface"
service_name = "simple_service"
config.hostname = "localhost"
config.port = 8097

class SimpleService(object):

    def __init__(self, service_name):
        self.service = rospy.Service(service_name, SetBool, self.handle_request)

    def handle_request(self, req):
        return SetBoolResponse(req.data, "flexxros!")

class ServiceClientWidget(ROSWidget):

    def callback(self, resp):
        self.receive_message.set_text(str(resp.success) + " " + resp.message)

    @flx.reaction("send_message.pointer_click")
    def _request_service(self, *events):
        self.call_service(service_name, "std_srvs/SetBool", {"data": self.type_message.checked}, self.callback)

    def init(self):
        with flx.FormLayout(flex=1, maxsize=400):
            self.type_message = flx.CheckBox(title="Checked?")
            self.send_message = flx.Button(text="Request service")
            self.receive_message = flx.Label(title="Response:", text="Waiting for response...")

class ServiceExampleInterface(ROSNode):

    def init(self):
        self.client = ServiceClientWidget()

rospy.init_node(node_name, anonymous=True)
service = SimpleService(service_name)
node.spin(ServiceExampleInterface)
```
