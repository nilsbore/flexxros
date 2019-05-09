#!/usr/bin/python3

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
        self.call_service(service_name, "std_srvs/SetBool", {"data": bool(int(self.type_message.text))}, self.callback)

    def init(self):
        with flx.FormLayout(flex=1, maxsize=400):
            self.type_message = flx.LineEdit(title="Request (0/1):", text="")
            self.send_message = flx.Button(text="Request service")
            self.receive_message = flx.Label(title="Response:", text="Waiting for response...")

class ServiceExampleInterface(ROSNode):

    def init(self):
        self.client = ServiceClientWidget()

rospy.init_node(node_name, anonymous=True)
service = SimpleService(service_name)
node.spin(ServiceExampleInterface)
