from flexx import flx
from flexxros.node import ROSWidget
from flexxros.widgets import ROSTopicPlotter, ROSDynReconfigWidget, ROSActionClientWidget

class ActuatorBox(ROSWidget):

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

        self.announce_publish(setpoint_topic, actuator_type)
        self.announce_publish(cont_setpoint_topic, "std_msgs/Float64")
        self.announce_publish(cont_enable_topic, "std_msgs/Bool")
        self.subscribe(setpoint_topic, actuator_type, self._setpoint_callback)
        self.subscribe(cont_setpoint_topic, "std_msgs/Float64", self._cont_callback)
        self.subscribe(cont_enable_topic, "std_msgs/Bool", self._enable_callback)

        if is_angles:
            self.reaction(self._setpoint_slider, "set_slider2.user_done")

    @flx.reaction("set_slider.user_done")
    def _setpoint_slider(self, *events):
        if self.is_angles:
            self.publish(self.setpoint_topic, {'weight_1_offset_radians': float(self.set_slider.value), 'weight_2_offset_radians': float(self.set_slider2.value)})
        else:
            self.publish(self.setpoint_topic, {'value': float(self.set_slider.value)})

    def _setpoint_callback(self, msg):
        if self.is_angles:
            self.set_slider.set_title(str(int(msg.weight_1_offset_radians)))
            self.set_slider.set_value(msg.weight_1_offset_radians)
            self.set_slider2.set_title(str(int(msg.weight_2_offset_radians)))
            self.set_slider2.set_value(msg.weight_2_offset_radians)
        else:
            self.set_slider.set_title(str(int(msg.value)))
            self.set_slider.set_value(msg.value)

    @flx.reaction("cont_slider.user_done")
    def _cont_slider(self, *events):
        self.publish(self.cont_setpoint_topic, {'data': float(self.cont_slider.value)})

    def _cont_callback(self, msg):
        self.cont_slider.set_title(str(int(msg.data)))
        self.cont_slider.set_value(msg.data)

    @flx.reaction("enable_cont.user_checked")
    def _enable_check(self, *events):
        self.publish(self.cont_enable_topic, {'data': bool(self.enable_cont.checked)})

    def _enable_callback(self, msg):
        self.enable_cont.set_checked(msg.data)

class SamPlots(flx.Widget):

    def init(self):

        with flx.HBox(flex=1):
            with flx.VBox(flex=1):
                self.plot1 = ROSTopicPlotter("/pitch_feedback", "std_msgs/Float64", "data", (-1.6, 1.6))
                flx.Widget(flex=1)
                self.plot2 = ROSTopicPlotter("/depth_feedback", "std_msgs/Float64", "data", (0, 6))
                flx.Widget(flex=1)
                self.plot3 = ROSTopicPlotter("/roll_feedback", "std_msgs/Float64", "data", (-1.6, 1.6))
            with flx.VBox(flex=1):
                self.plot4 = ROSTopicPlotter("/uavcan_lcg_command", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot5 = ROSTopicPlotter("/uavcan_vbs_command", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot6 = ROSTopicPlotter("/ros_to_uavcan_bridge_node/tcg_command1", "sam_msgs/BallastAngles", "weight_1_offset_radians", (-3.14, 3.14))
            with flx.VBox(flex=1):
                self.plot7 = ROSTopicPlotter("/uavcan_to_ros_bridge_node/lcg_feedback", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot8 = ROSTopicPlotter("/uavcan_to_ros_bridge_node/vbs_feedback", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                flx.Widget(minsize=220)

class SamActuatorBar(ROSWidget):

    def init(self):

        with flx.VBox(flex=0, minsize=300, style="background: #9d9;"):
            lcg_actuator = ActuatorBox("Pitch - LCG", "sam_msgs/PercentStamped",
                                       "/uavcan_lcg_command", "/uavcan_to_ros_bridge_node/lcg_feedback",
                                       "/LCG_trim/pid_enable", "/pitch_setpoint", -1.6, 1.6)
            vbs_actuator = ActuatorBox("Depth - VBS", "sam_msgs/PercentStamped",
                                       "/uavcan_vbs_command", "/uavcan_to_ros_bridge_node/vbs_feedback",
                                       "/VBS_depth/pid_enable", "/depth_setpoint", 0, 5)
            tcg_actuator = ActuatorBox("Roll - TCG", "sam_msgs/BallastAngles",
                                       "/ros_to_uavcan_bridge_node/tcg_command1", "",
                                       "/TCG_pid/pid_enable", "/roll_setpoint", -1.6, 1.6, True)
            self.leak_button = flx.Button(text="No leaks...", style="background: #008000;", disabled=True)

            flx.Widget(flex=1)

            self.startup_check = ROSActionClientWidget("/sam_startup_check", "sam_msgs/SystemsCheck")
            self.abort_button = flx.Button(text="Abort", style="background: #ff6961;")

            self.subscribe("/uavcan_leak", "sam_msgs/Leak", self.callback)

    def callback(msg):

        if msg.value:
            self.leak_button.set_text("Leaking!")
            self.leak_button.apply_style("background: #ff6961;")

    @flx.reaction('abort_button.pointer_click')
    def _publish_abort(self, *events):
        self.publish("/abort", {})
