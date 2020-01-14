from flexx import flx
from flexxros.node import ROSNode, ROSWidget
import rospy
import rosmon_msgs.msg
import rosmon_msgs.srv

class ROSMonNodeWidget(ROSWidget):
    
    node_states = ["IDLE", "RUNNING", "CRASHED", "WAITING"]
    node_actions = {"START": 1, "STOP": 2, "RESTART": 3}

    def init(self, service_name):

        self.service_name = service_name
        self.node_name = ""
        self.node_ns = ""

        with flx.HFix():
            self.name = flx.Label(text="AA")
            self.state = flx.Label(text="BB")
            self.user_load = flx.Label(text="CC")
            self.memory = flx.Label(text="DD")
            self.restarts = flx.Label(text="EE")
            self.start_stop = flx.Button(text="Stop")

    def callback(self, resp):
        pass

    @flx.reaction("start_stop.pointer_click")
    def _request_service(self, *events):
        if not self.node_name:
            return
        if self.start_stop.text == "Stop":
            action = self.node_actions["STOP"]
        else:
            action = self.node_actions["START"]
        req = {"node": self.node_name, "ns": self.node_ns, "action": action}
        self.call_service(self.service_name, "rosmon_msgs/StartStop", req, self.callback)

    def set_values(self, nstate):

        self.node_name = nstate.name
        self.node_ns = nstate.ns
        self.name.set_text(nstate.name)
        current_state = self.node_states[nstate.state]
        self.state.set_text(current_state)
        self.user_load.set_text("%.02f%" % (100.*nstate.user_load))
        self.memory.set_text("%.02f MB" % (1e-6*float(nstate.memory)))
        self.restarts.set_text(str(nstate.restart_count))
        if current_state == "RUNNING":
            self.start_stop.set_text("Stop")
            self.state.apply_style('color: #0F0;')
        else:
            self.start_stop.set_text("Start")
            self.state.apply_style('color: #F00;')

class ROSMonLaunchWidget(ROSWidget):

    def callback(self, msg):

        action_text = "Start"
        state_text = "IDLE"
        for nstate in msg.nodes:
            if nstate.name not in self.nodes:
                with self.base_layout:
                    self.nodes[nstate.name] = ROSMonNodeWidget(self.service_name)
            self.nodes[nstate.name].set_values(nstate)
            if nstate.state == 1:
                action_text = "Stop"
                state_text = "RUNNING"
        self.start_stop.set_text(action_text)
        self.launch_state.set_text(state_text)

        if state_text == "RUNNING":
            self.launch_state.apply_style('color: #0F0;')
        else:
            self.launch_state.apply_style('color: #F00;')

    @flx.reaction("start_stop.pointer_click")
    def _request_service(self, *events):

        if self.start_stop.text == "Start":
            self.launch_state.set_text("STARTING...")
            self.root.rosmon_launch(self.service_name, 1) #ROSMonNodeWidget.node_actions["STOP"])
        else:
            self.launch_state.set_text("STOPPING...")
            self.root.rosmon_launch(self.service_name, 2) #ROSMonNodeWidget.node_actions["STOP"])

    @flx.reaction("expand.pointer_click")
    def _expand(self, *events):

        if self.base_layout.parent is None:
            self.base_layout.set_parent(self.list_container)
            #self.expand.set_text("Collapse")
            self.expand.set_text("v")
        else:
            self.base_layout.set_parent(None)
            #self.expand.set_text("Expand")
            self.expand.set_text(">")

    def init(self, topic):

        self.server_name = topic.split("/")[1]
        self.service_name = "/" + self.server_name + "/start_stop"

        self.nodes = {}
        self.subscribe(topic, "rosmon_msgs/State", self.callback)
        self.list_container = flx.VBox(flex=1)
        with self.list_container:
            with flx.HBox(flex=1):
                self.expand = flx.Button(flex=0, text=">")
                self.label = flx.Label(flex=0, text=self.server_name + ":")
                self.launch_state = flx.Label(flex=0, text="IDLE", style='color: #F00;')
                #self.expand = flx.Button(text="Expand")
                flx.Widget(flex=1)
                self.start_stop = flx.Button(flex=0, text="Stop")
            self.base_layout = flx.VBox(flex=1, style='border:1px solid #777;')
            with self.base_layout:
                with flx.HFix():
                    flx.Label(text="Name")
                    flx.Label(text="State")
                    flx.Label(text="CPU")
                    flx.Label(text="Memory")
                    flx.Label(text="Restarts")
                    flx.Label(text="Action")
        self.base_layout.set_parent(None)

class ROSMonDashboardWidget(flx.Widget):

    @flx.reaction("!root.add_launch")
    def _add_launch(self, *events):

        for ev in events:
            with self.base_layout:
                ROSMonLaunchWidget(ev.topic)

    def init(self):
        with flx.VBox(flex=1):
            self.base_layout = flx.VBox(flex=0)
            flx.Widget(flex=1)

class ROSMonNode(ROSNode):

    def __init__(self,  *args, **kwargs):
        super().__init__( *args, **kwargs)
        self.feedback_topics = {}
        self.timer = rospy.Timer(rospy.Duration(1.0), self.maybe_subscribe)

    @flx.action
    def rosmon_launch(self, service_name, action):

        if action == 2: # "STOP"
            desired_state = 0 # "IDLE"
        else:
            desired_state = 1 # "RUNNING"

        topic = "/" + service_name.split("/")[1] + "/state"

        msg = rospy.wait_for_message(topic, rosmon_msgs.msg.State, timeout=None)
        for nstate in msg.nodes:

            # If not in desired state, or if not desired state is IDLE (0) and node CRASHED (2)
            if nstate.state != desired_state and not (desired_state == 0 and nstate.state == 2):
                rospy.wait_for_service(service_name)
                start_stop = rospy.ServiceProxy(service_name, rosmon_msgs.srv.StartStop)
                try:
                    resp = start_stop(nstate.name, nstate.ns, action)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

            new_state = nstate
            # If not in desired state, or if not desired state is IDLE (0) and node CRASHED (2)
            while new_state.state != desired_state and not (desired_state == 0 and new_state.state == 2):
                new_msg = rospy.wait_for_message(topic, rosmon_msgs.msg.State, timeout=None)
                new_state = next(n for n in new_msg.nodes if n.name == nstate.name)

    def maybe_subscribe(self, events):

        topics = rospy.get_published_topics()
        topics.sort()

        for topic in topics:
            if topic[1] != "rosmon_msgs/State" or topic[0] in self.feedback_topics:
                continue
            self.feedback_topics[topic[0]] = 0
            self.emit("add_launch", {"topic": topic[0]})
