#!/usr/bin/python3

from flexx import flx, config
import rospy
from flexxros import ROSNode, ROSTopicPlotter, ROSDynReconfigWidget #, relay
import flexxros
import time

import asyncio
import tornado.platform.asyncio as torasync

class SamPlots(flx.Widget):

    def init(self):

        with flx.HBox(flex=1):
            with flx.VBox(flex=1):
                self.plot1 = ROSTopicPlotter("/pitch_feedback", "std_msgs/Float64", "data", (-1.6, 1.6))
                flx.Widget(flex=1)
                self.plot2 = ROSTopicPlotter("/roll_feedback", "std_msgs/Float64", "data", (-1.6, 1.6))
                flx.Widget(flex=1)
                self.plot3 = ROSTopicPlotter("/depth_feedback", "std_msgs/Float64", "data", (0, 6))
            with flx.VBox(flex=1):
                self.plot4 = ROSTopicPlotter("/uavcan_lcg_command", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot5 = ROSTopicPlotter("/ros_to_uavcan_bridge_node/tcg_command1", "sam_msgs/BallastAngles", "weight_1_offset_radians", (-3.14, 3.14))
                flx.Widget(flex=1)
                self.plot6 = ROSTopicPlotter("/uavcan_vbs_command", "sam_msgs/PercentStamped", "value")
            with flx.VBox(flex=1):
                self.plot7 = ROSTopicPlotter("/pitch_setpoint", "std_msgs/Float64", "data", (-1.6, 1.6))
                flx.Widget(flex=1)
                self.plot8 = ROSTopicPlotter("/roll_setpoint", "std_msgs/Float64", "data", (-1.6, 1.6))
                flx.Widget(flex=1)
                self.plot9 = ROSTopicPlotter("/depth_setpoint", "std_msgs/Float64", "data", (0, 6))

class SamDashboard(flx.Widget):

    def init(self):
        
        with flx.HBox():
            with flx.VBox(flex=0, minsize=250, style="background: #9d9;"):
                flx.Widget(minsize=20)
                flx.Label(text='Pitch setpoint:') 
                self.pitch_setpoint = flx.LineEdit(text='0', maxsize=(600, 0))
                self.pitch_button = flx.Button(text="Publish setpoint")
                flx.Widget(minsize=20)
                flx.Label(text='Roll setpoint:') 
                self.roll_setpoint = flx.LineEdit(text='0', maxsize=(600, 0))
                self.roll_button = flx.Button(text="Publish setpoint")
                flx.Widget(minsize=20)
                flx.Label(text='Depth setpoint:') 
                self.depth_setpoint = flx.LineEdit(text='0', maxsize=(600, 0))
                self.depth_button = flx.Button(text="Publish setpoint")
                flx.Widget(flex=1)
                self.reconfig = ROSDynReconfigWidget("/LCG_trim/controller")
                self.reconfig = ROSDynReconfigWidget("/VBS_depth/controller")

            plots = SamPlots(flex=1)

        self.root.announce_publish("/pitch_setpoint", "std_msgs/Float64")
        self.root.announce_publish("/roll_setpoint", "std_msgs/Float64")
        self.root.announce_publish("/depth_setpoint", "std_msgs/Float64")
        #flexxros.node.announce_publish("/pitch_setpoint", "std_msgs/Float64")
        #flexxros.node.announce_publish("/roll_setpoint", "std_msgs/Float64")
        #flexxros.announce_publish("/depth_setpoint", "std_msgs/Float64")

    @flx.reaction('depth_button.pointer_click')
    def _publish_depth(self, *events):
        self.root.publish("/depth_setpoint", {'data': float(self.depth_setpoint.text)})
        #flexxros.node.publish("/depth_setpoint", {'data': float(self.depth_setpoint.text)})

    @flx.reaction('pitch_button.pointer_click')
    def _publish_pitch(self, *events):
        self.root.publish("/pitch_setpoint", {'data': float(self.pitch_setpoint.text)})
        #flexxros.node.publish("/pitch_setpoint", {'data': float(self.pitch_setpoint.text)})

    @flx.reaction('roll_button.pointer_click')
    def _publish_roll(self, *events):
        self.root.publish("/roll_setpoint", {'data': float(self.roll_setpoint.text)})
        #flexxros.node.publish("/roll_setpoint", {'data': float(self.roll_setpoint.text)})

class ROSInterface(ROSNode):

    def init(self):

        self.main_widget = SamDashboard()

if __name__ == '__main__':

    config.hostname = "130.237.218.207"
    config.port = 8097

    #rospy.init_node("web_interface", anonymous=True)
    #asyncio.set_event_loop_policy(torasync.AnyThreadEventLoopPolicy())
    #app = flx.App(ROSInterface)
    #app.serve('')
    #flx.start()
    #m = flx.launch(ROSInterface, 'app')
    #flx.run()
    
    flexxros.init_and_spin("web_interface", ROSInterface)

