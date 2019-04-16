#!/usr/bin/python3

from flexx import flx, config
from flexxros import flexxros
from flexxros.flexxros import ROSNode
from flexxros.sam_widgets import SamActuatorBar

class SamControls(flx.Widget):

    def init(self):
        
        with flx.HBox():
            actuator_bar = SamActuatorBar()
            flx.Widget(flex=1)

class ROSInterface(ROSNode):

    def init(self):

        self.main_widget = SamControls()

if __name__ == '__main__':

    config.hostname = "130.237.36.51"
    config.port = 8097
    
    flexxros.init_and_spin("web_interface", ROSInterface)

