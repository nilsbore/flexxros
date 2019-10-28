#!/usr/bin/python3

from flexx import flx, config
from flexxros import node
from flexxros.rosmon import ROSMonNode, ROSMonDashboardWidget
import rospy

node_name = "rosmon_dashboard_interface"
config.hostname = "localhost"
config.port = 8097

class ROSMonDasboardInterface(ROSMonNode):

    def init(self):
        self.dashboard = ROSMonDashboardWidget()

rospy.init_node(node_name, anonymous=True)
node.spin(ROSMonDasboardInterface)
