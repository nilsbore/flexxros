#!/usr/bin/python3

from flexx import flx, config
from flexxros import node
from flexxros.node import ROSNode
from flexxros.sam_widgets import SamActuatorBar, SamPlots
from flexxros.ros3djs_widgets import RobotModelWidget

import rospy

class SamDashboard(flx.Widget):

    def init(self, host_ip, base_link, robot_description,
                   rosbridge_port, resources_port):
        
        with flx.HBox():

            actuator_bar = SamActuatorBar()
            robot_model = RobotModelWidget(flex=1, host_ip=host_ip,
                                           base_link=base_link,
                                           robot_description=robot_description,
                                           rosbridge_port=rosbridge_port,
                                           resources_port=resources_port)

class ROSInterface(ROSNode):

    def init(self):

        host_ip = rospy.get_param('~host_ip', "127.0.0.1")
        base_link = rospy.get_param('~base_link', "/base_link")
        robot_description = rospy.get_param('~robot_description', "/flexxros_description")
        rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
        resources_port = rospy.get_param('~resources_port', 9091)

        self.main_widget = SamDashboard(host_ip, base_link, robot_description,
                                        rosbridge_port, resources_port)

if __name__ == '__main__':

    rospy.init_node("web_interface", anonymous=True)

    config.hostname = rospy.get_param('~host_ip', "127.0.0.1")
    config.port = rospy.get_param('~app_port', 8097)

    node.spin(ROSInterface)
