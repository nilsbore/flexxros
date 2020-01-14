#!/usr/bin/python3

from flexx import flx, config
from flexxros import node
from flexxros.node import ROSNode
from flexxros.sam_widgets import SamActuatorBar, SamInfoDash
from flexxros.rosmon import ROSMonNode, ROSMonDashboardWidget
import rospy

class SamControls(flx.Widget):

    def init(self):
        
        with flx.HBox():
            actuator_bar = SamActuatorBar()
            #flx.Widget(flex=1)
            with flx.VBox(flex=1):
                launch_dashboard = ROSMonDashboardWidget(flex=1)
                info_dash = SamInfoDash(flex=0)


class ROSInterface(ROSMonNode):

    def init(self):

        self.main_widget = SamControls()

if __name__ == '__main__':

    rospy.init_node("web_interface", anonymous=True)

    config.hostname = rospy.get_param("~display_ip", "192.168.2.61")
    config.port = 8097
    
    node.spin(ROSInterface)

