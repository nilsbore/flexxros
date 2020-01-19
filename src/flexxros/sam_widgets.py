from flexx import flx
from flexxros.node import ROSWidget
from flexxros.widgets import ROSTopicPlotter, ROSDynReconfigWidget, ROSActionClientWidget

class GenericActuatorBox(ROSWidget):

    def init(self, name, topic, topic_type, actuator_settings):
        self.topic = topic
        with flx.GroupWidget(title=name, flex=1):
            with flx.FormLayout(flex=1):
                self.sliders = []
                self.members = []
                for settings in actuator_settings:
                    self.sliders.append(flx.Slider(title=settings["name"], min=settings["min"], max=settings["max"], value=0))
                    if "type" in settings:
                        self.members.append((settings["member"], settings["type"]))
                    else:
                        self.members.append((settings["member"], "float"))
                #flx.Widget(minsize=10)

        self.announce_publish(topic, topic_type)
        self.subscribe(topic, topic_type, self._setpoint_callback)

    @flx.reaction("sliders*.user_done")
    def _setpoint_slider(self, *events):
        msg = {}
        for mem, slider in zip(self.members, self.sliders):
            if mem[1] == "int":
                msg[mem[0]] = int(slider.value)
            else:
                msg[mem[0]] = slider.value
        self.publish(self.topic, msg)

    def _setpoint_callback(self, msg):
        for mem, slider in zip(self.members, self.sliders):
            slider.set_value(msg[mem[0]])


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
                self.plot4 = ROSTopicPlotter("/sam/core/lcg_cmd", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot5 = ROSTopicPlotter("/sam/core/vbs_cmd", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot6 = ROSTopicPlotter("/sam/core/tcg_cmd", "sam_msgs/BallastAngles", "weight_1_offset_radians", (-3.14, 3.14))
            with flx.VBox(flex=1):
                self.plot7 = ROSTopicPlotter("/sam/core/lcg_fb", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                self.plot8 = ROSTopicPlotter("/sam/core/vbs_fb", "sam_msgs/PercentStamped", "value")
                flx.Widget(flex=1)
                flx.Widget(minsize=220)

class SamInfoDash(ROSWidget):

    def init(self):

        with flx.HBox(flex=1, style="background: #e6e6df;"):
            with flx.FormLayout(flex=1):
                flx.Widget(minsize=20)
                self.heading = flx.LineEdit(title="Heading", text="")
                self.pitch = flx.LineEdit(title="Pitch", text="")
                self.roll = flx.LineEdit(title="Roll", text="")
                flx.Widget(minsize=40)
            with flx.FormLayout(flex=1):
                flx.Widget(minsize=20)
                self.depth = flx.LineEdit(title="Depth", text="")
                self.xpos = flx.LineEdit(title="X", text="")
                self.ypos = flx.LineEdit(title="Y", text="")
                flx.Widget(minsize=40)
            with flx.FormLayout(flex=1):
                flx.Widget(minsize=20)
                self.xvel = flx.LineEdit(title="X vel", text="")
                self.yvel = flx.LineEdit(title="Y vel", text="")
                self.zvel = flx.LineEdit(title="Z vel", text="")
                flx.Widget(minsize=40)
            with flx.FormLayout(flex=1):
                flx.Widget(minsize=20)
                self.gps_status = flx.LineEdit(title="GPS Status", text="")
                self.dvl_status = flx.LineEdit(title="DVL Status", text="")
                self.battery_status = flx.LineEdit(title="Battery level", text="")
                flx.Widget(minsize=40)
            with flx.FormLayout(flex=1):
                flx.Widget(minsize=20)
                self.vbs_fb = flx.LineEdit(title="VBS fb", text="")
                self.lcg_fb = flx.LineEdit(title="LCG fb", text="")
                self.rpm_fb = flx.LineEdit(title="RPM fb", text="")
                flx.Widget(minsize=40)
            
        # We subscribe to these topics at full frquency (no extra arg)
        self.subscribe("/sam/core/gps", "sensor_msgs/NavSatFix", self.gps_callback)
        self.subscribe("/sam/core/battery_fb", "sensor_msgs/BatteryState", self.battery_callback)
        # We only subscribe to these topics at 1hz
        self.subscribe("/sam/dr/odom", "nav_msgs/Odometry", self.odom_callback, 1.)
        self.subscribe("/sam/core/vbs_fb", "sam_msgs/PercentStamped", self.vbs_callback, 1.)
        self.subscribe("/sam/core/lcg_fb", "sam_msgs/PercentStamped", self.lcg_callback, 1.)
        self.subscribe("/sam/ctrl/depth_feedback", "std_msgs/Float64", self.depth_callback, 1.)
        self.subscribe("/sam/ctrl/pitch_feedback", "std_msgs/Float64", self.pitch_callback, 1.)
        self.subscribe("/sam/ctrl/roll_feedback", "std_msgs/Float64", self.roll_callback, 1.)
        self.subscribe("/sam/ctrl/yaw_feedback", "std_msgs/Float64", self.yaw_callback, 1.)

    def odom_callback(self, msg):

        self.xpos.set_text("%.02fm" % msg.pose.pose.position.x)
        self.ypos.set_text("%.02fm" % msg.pose.pose.position.y)
        self.xvel.set_text("%.02fm/s" % msg.twist.twist.linear.x)
        self.yvel.set_text("%.02fm/s" % msg.twist.twist.linear.y)
        self.zvel.set_text("%.02fm/s" % msg.twist.twist.linear.z)

    def vbs_callback(self, msg):

        self.vbs_fb.set_text("%.02f%" % msg.value)

    def lcg_callback(self, msg):

        self.lcg_fb.set_text("%.02f%" % msg.value)

    def depth_callback(self, msg):

        self.depth.set_text("%.02fm" % msg.data)

    def pitch_callback(self, msg):

        self.pitch.set_text("%.02f" % (180./3.14*msg.data))

    def roll_callback(self, msg):

        self.roll.set_text("%.02f" % (180./3.14*msg.data))

    def yaw_callback(self, msg):

        self.heading.set_text("%.02f" % (90. - 180./3.14*msg.data))

    def battery_callback(self, msg):
        
        # battery health not good
        if msg.power_supply_health != 1 or msg.percentage < 20.:
            self.battery_status.apply_style("background: #ffb3af;")
        else:
            self.battery_status.apply_style("background: #bbffbb;")
        self.battery_status.set_text("%.02f%" % msg.percentage)

    def gps_callback(self, msg):
        
        # no fix
        if msg.status.status == -1:
            self.gps_status.apply_style("background: #ffb3af;")
        else:
            self.gps_status.apply_style("background: #bbffbb;")
            self.gps_status.set_text("Lat: %.04f, Lon: %.04f" % (msg.latitude, msg.longitude))

class SamActuatorBar(ROSWidget):

    def init(self):

        with flx.VBox(flex=0, minsize=400, style="background: #9d9;"):
            self.thruster_angles = GenericActuatorBox("Thruster Angles", "/sam/core/thrust_vector_cmd", "sam_msgs/ThrusterAngles",
                                                      [{"name": "Hori.", "member": "thruster_horizontal_radians", "min": -0.1, "max": 0.18},
                                                       {"name": "Vert.", "member": "thruster_vertical_radians", "min": -0.1, "max": 0.15}])
            self.thruster_rpms = GenericActuatorBox("Thruster RPMs", "/sam/core/rpm_cmd", "sam_msgs/ThrusterRPMs",
                                                    [{"name": "Front", "member": "thruster_1_rpm", "min": -1000, "max": 1000, "type": "int"},
                                                     {"name": "Back", "member": "thruster_2_rpm", "min": -1000, "max": 1000, "type": "int"}])
            self.leak_button = flx.Button(text="No leaks...", style="background: #008000;", disabled=True)
            lcg_actuator = ActuatorBox("Pitch - LCG", "sam_msgs/PercentStamped",
                                       "/sam/core/lcg_cmd", "/sam/core/lcg_fb",
                                       "/sam/ctrl/lcg/pid_enable", "/sam/ctrl/lcg/setpoint", -1.6, 1.6)
            vbs_actuator = ActuatorBox("Depth - VBS", "sam_msgs/PercentStamped",
                                       "/sam/core/vbs_cmd", "/sam/core/vbs_fb",
                                       "/sam/ctrl/vbs/pid_enable", "/sam/ctrl/vbs/setpoint", 0, 5)
            tcg_actuator = ActuatorBox("Roll - TCG", "sam_msgs/BallastAngles",
                                       "/sam/core/tcg_cmd", "",
                                       "/sam/ctrl/tcg/pid_enable", "/sam/ctrl/tcg/setpoint", -1.6, 1.6, True)


            #flx.Widget(flex=1)
            with flx.TabLayout(flex=1):
                self.startup_check = ROSActionClientWidget("/sam/startup_check", "sam_msgs/SystemsCheck", title="Startup check", flex=1)
                self.gps_fix_action = ROSActionClientWidget("/sam/gps_fix_server", "sam_msgs/GetGPSFix", title="Get GPS fix", flex=1)

            self.abort_button = flx.Button(text="Abort", style="background: #ff6961;")

            self.subscribe("/sam/core/leak_fb", "sam_msgs/Leak", self.leak_callback)
            self.announce_publish("/abort", "std_msgs/Empty")

    def leak_callback(msg):

        if msg.value:
            self.leak_button.set_text("Leaking!")
            self.leak_button.apply_style("background: #ff6961;")

    @flx.reaction('abort_button.pointer_click')
    def _publish_abort(self, *events):
        print("Abort button was clicked!")
        self.publish("/abort", {})
