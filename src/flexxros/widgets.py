from flexx import flx
from flexxros.node import ROSWidget

class ROSTopicPlotter(ROSWidget):
    """
    flx.Widget subclass that subscribes to a topic and plots it using a flx.PlotWidget
    """

    def init(self, topic, topic_type, key="data", yrange=(0, 100), nsamples=100):
        self.key = key
        self.nsamples = nsamples
        self.start_time = time()
        self.plot = self.mem_plot = flx.PlotWidget(flex=1, style='width: 400px; height: 220px;',
                                                   xdata=[], yrange=yrange, ylabel=topic+"/"+key, xlabel="Time")
        self.subscribe(topic, topic_type, self._callback)

    def _callback(self, msg):

        #try:
        # Prepare plots
        times = list(self.plot.xdata)
        times.append(time() - self.start_time)
        times = times[-self.nsamples:]
        
        # cpu data
        values = list(self.plot.ydata)
        values.append(msg[self.key])
        values = values[-self.nsamples:]
        self.plot.set_data(times, values)

        #except BufferError:
        #    print("Got buffer error!")

class ROSDynReconfigWidget(flx.Widget):
    """
    flx.Widget subclass that dynamically creates a form to set parameters through dynamic reconfigure
    """

    def init(self, server_name):

        self.is_init = False
        self.react = self.reaction(self._callback, "!root."+server_name.replace("/", "_"))

        with flx.GroupWidget(title=server_name):
            self.vbox = flx.FormLayout(flex=1)

        self.root.announce_reconfig(server_name)

    @flx.action
    def add_children(self, ev):

        print("Got add children!")

        if self.is_init:
            for c in ev:
                for child in self.vbox.children:
                    print(child.title, child.text)
                    if child.title == c:
                        child.set_text(str(ev[c]))
                        break
            return

        print("Event: ", ev)

        with self.vbox:
            for c in ev:
                if c in ["groups", "type", "source"]:
                    continue
                l = flx.LineEdit(title=c, text=str(ev[c]))
            flx.Button(text="Set")
            flx.Widget(minsize=60)

        self.is_init = True

    def _callback(self, *events):

        print("Got new reconfig!")

        for ev in events:
            self.add_children(ev)

class ROSActionClientWidget(ROSWidget):
    """
    flx.Widget subclass that presents a widget similar to the normal axclient.py
    """

    def init(self, server_name, server_type):

        self.is_init = False
        self.server_name = server_name

        with flx.GroupWidget(title=server_name, flex=1):
            with flx.FormLayout(flex=1):
                self.arguments = flx.LineEdit(title="Args", text="")
                self.feedback = flx.LineEdit(title="Feedback", text="")
                self.result = flx.LineEdit(title="Result", text="")
                self.send_goal = flx.Button(text="Send goal")
                flx.Widget(minsize=40)

        self.reaction(self._prototype_callback, "!root."+server_name.replace("/", "_")+"_prototype")
        self.announce_action_client(server_name, server_type)

    @flx.reaction("send_goal.pointer_click")
    def _send_goal(self, *events):
        self.send_action_goal(self.server_name, self.arguments.text, self._feedback_callback, self._result_callback)
        self.feedback.set_text("Waiting, is %s running?" % self.server_name)
        self.result.set_text("Waiting...")

    def _prototype_callback(self, msg_string):
        self.arguments.set_text(msg_string.data)

    def _feedback_callback(self, msg):

        print("Got new feedback!")
        exclude = ["source", "type"]
        texts = [str(key) + ": " + str(value) for key, value in msg.items() if key not in exclude]
        self.feedback.set_text(", ".join(texts))

    def _result_callback(self, msg):

        print("Got new result!")
        exclude = ["source", "type"]
        texts = [str(key) + ": " + str(value) for key, value in msg.items() if key not in exclude]
        self.result.set_text(", ".join(texts))
        self.feedback.set_text("")

