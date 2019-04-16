from flexx import flx, config
import rospy
from flexxros import ROSNode, ROSTopicPlotter, ROSDynReconfigWidget, ROSActionClientWidget #, relay
import flexxros
import time

import asyncio
import tornado.platform.asyncio as torasync

class ActuatorBox(flx.Widget):

    def init(self, name, actuator_type, setpoint_topic,
             feedback_topic, cont_enable_topic,
             cont_setpoint_topic, cont_min, cont_max,
             is_angles=False):

        self.setpoint_topic = setpoint_topic
        self.cont_setpoint_topic = cont_setpoint_topic
        self.cont_enable_topic = cont_enable_topic
        self.is_angles = is_angles

        with flx.GroupWidget(title=name, flex=1):
            with flx.FormLayout(flex=1):
                flx.Label(text="Actuator setpoint")
                if is_angles:
                    self.set_slider = flx.Slider(title="0", min=-1.6, max=1.6)
                    self.set_slider2 = flx.Slider(title="0", min=-1.6, max=1.6)
                else:
                    self.set_slider = flx.Slider(title="50", min=0, max=100, value=50)
                flx.Label(text="Controller setpoint")
                self.enable_cont = flx.CheckBox(title="Enabled")
                self.enable_cont.set_checked(True)
                self.cont_slider = flx.Slider(title="0", min=cont_min, max=cont_max, value=0)
                flx.Widget(minsize=20)

        self.root.announce_publish(setpoint_topic, actuator_type)
        self.root.announce_publish(cont_setpoint_topic, "std_msgs/Float64")
        self.root.announce_publish(cont_enable_topic, "std_msgs/Bool")
        self.root.subscribe(setpoint_topic, actuator_type)
        self.root.subscribe(cont_setpoint_topic, "std_msgs/Float64")
        self.root.subscribe(cont_enable_topic, "std_msgs/Bool")

        self.setpoint_react = self.reaction(self._setpoint_callback, "!root."+setpoint_topic.replace("/", "_"))
        self.cont_react = self.reaction(self._cont_callback, "!root."+cont_setpoint_topic.replace("/", "_"))
        self.enable_react = self.reaction(self._enable_callback, "!root."+cont_enable_topic.replace("/", "_"))

        if is_angles:
            self.setpoint2_react = self.reaction(self._setpoint_slider, "set_slider2.user_done")

    @flx.reaction("set_slider.user_done")
    def _setpoint_slider(self, *events):

        if self.is_angles:
            self.root.publish(self.setpoint_topic, {'weight_1_offset_radians': float(self.set_slider.value), 'weight_2_offset_radians': float(self.set_slider2.value)})
        else:
            self.root.publish(self.setpoint_topic, {'value': float(self.set_slider.value)})

    def _setpoint_callback(self, *events):

        if self.is_angles:
            for ev in events:
                self.set_slider.set_title(str(int(ev.weight_1_offset_radians)))
                self.set_slider.set_value(ev.weight_1_offset_radians)
                self.set_slider2.set_title(str(int(ev.weight_2_offset_radians)))
                self.set_slider2.set_value(ev.weight_2_offset_radians)
        else:
            for ev in events:
                self.set_slider.set_title(str(int(ev.value)))
                self.set_slider.set_value(ev.value)

    @flx.reaction("cont_slider.user_done")
    def _cont_slider(self, *events):

        self.root.publish(self.cont_setpoint_topic, {'data': float(self.cont_slider.value)})

    def _cont_callback(self, *events):

        for ev in events:
            self.cont_slider.set_title(str(int(ev.data)))
            self.cont_slider.set_value(ev.data)

    @flx.reaction("enable_cont.user_checked")
    def _enable_check(self, *events):

        self.root.publish(self.cont_enable_topic, {'data': bool(self.enable_cont.checked)})

    def _enable_callback(self, *events):

        for ev in events:
            self.enable_cont.set_checked(ev.data)

class SamPlots(flx.Widget):

    def init(self):

        with flx.HBox(flex=1):
            with flx.VBox(flex=1):
                self.plot1 = ROSTopicPlotter("/pitch_feedback", "std_msgs/Float64", "data", (-1.6, 1.6))
                flx.Widget(flex=1)
                self.plot3 = ROSTopicPlotter("/depth_feedback", "std_msgs/Float64", "data", (0, 6))
                flx.Widget(flex=1)
                self.plot2 = ROSTopicPlotter("/roll_feedback", "std_msgs/Float64", "data", (-1.6, 1.6))
            with flx.VBox(flex=1):
                self.plot4 = ROSTopicPlotter("/uavcan_lcg_command", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot6 = ROSTopicPlotter("/uavcan_vbs_command", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot5 = ROSTopicPlotter("/ros_to_uavcan_bridge_node/tcg_command1", "sam_msgs/BallastAngles", "weight_1_offset_radians", (-3.14, 3.14))
            with flx.VBox(flex=1):
                self.plot10 = ROSTopicPlotter("/uavcan_to_ros_bridge_node/lcg_feedback", "sam_msgs/PercentStamped", "value")
                flx.Widget(minsize=40)
                self.plot11 = ROSTopicPlotter("/uavcan_to_ros_bridge_node/vbs_feedback", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)

class SamActuatorBar(flx.Widget):

    def init(self):

        with flx.VBox(flex=0, minsize=300, style="background: #9d9;"):
            lcg_actuator = ActuatorBox("Pitch - LCG", "sam_msgs/PercentStamped",
                                       "/uavcan_lcg_command", "/uavcan_to_ros_bridge_node/lcg_feedback",
                                       "/LCG_pid/pid_enable", "/pitch_setpoint", -1.6, 1.6)
            vbs_actuator = ActuatorBox("Depth - VBS", "sam_msgs/PercentStamped",
                                       "/uavcan_vbs_command", "/uavcan_to_ros_bridge_node/vbs_feedback",
                                       "/VBS_pid/pid_enable", "/depth_setpoint", 0, 5)
            tcg_actuator = ActuatorBox("Roll - TCG", "sam_msgs/BallastAngles",
                                       "/ros_to_uavcan_bridge_node/tcg_command1", "",
                                       "/TCG_pid/pid_enable", "/roll_setpoint", -1.6, 1.6, True)

            flx.Widget(flex=1)

            self.startup_check = ROSActionClientWidget("/sam_startup_check", "sam_msgs/SystemsCheck")
            self.abort_button = flx.Button(text="Abort", style="background: #ff6961;")

    @flx.reaction('abort_button.pointer_click')
    def _publish_abort(self, *events):
        self.root.publish("/abort", {})
