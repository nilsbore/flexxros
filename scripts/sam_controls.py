#!/usr/bin/python3

from flexx import flx, config
from flexxros import node
from flexxros.node import ROSNode
from flexxros.sam_widgets import SamActuatorBar
from flexxros.rosmon import ROSMonNode, ROSMonDashboardWidget
import rospy

class SamControls(flx.Widget):

    def init(self):
        
        with flx.HBox():
            actuator_bar = SamActuatorBar()
            launch_dashboard = ROSMonDashboardWidget(flex=1)
            #flx.Widget(flex=1)

class ROSInterface(ROSMonNode):

    def init(self):

        self.main_widget = SamControls()

if __name__ == '__main__':

    rospy.init_node("web_interface", anonymous=True)

    config.hostname = rospy.get_param("~display_ip", "130.237.36.51")
    config.port = 8097
    
    node.spin(ROSInterface)

