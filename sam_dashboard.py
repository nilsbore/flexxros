#!/usr/bin/python3

from flexx import flx, config
from flexxros import flexxros
from flexxros.flexxros import ROSNode
from flexxros.sam_widgets import SamActuatorBar, SamPlots

#import rospy
#import asyncio
#import tornado.platform.asyncio as torasync

class SamDashboard(flx.Widget):

    def init(self):
        
        with flx.HBox():

            actuator_bar = SamActuatorBar()
            plots = SamPlots(flex=1)

class ROSInterface(ROSNode):

    def init(self):

        self.main_widget = SamDashboard()

if __name__ == '__main__':

    config.hostname = "localhost"
    config.port = 8097

    #rospy.init_node("web_interface", anonymous=True)
    #asyncio.set_event_loop_policy(torasync.AnyThreadEventLoopPolicy())
    #app = flx.App(ROSInterface)
    #app.serve('')
    #flx.start()
    #m = flx.launch(ROSInterface, 'app')
    #flx.run()
    
    flexxros.init_and_spin("web_interface", ROSInterface)

