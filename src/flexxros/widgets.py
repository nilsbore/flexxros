from flexx import flx
from flexxros.node import ROSWidget

class ROSTopicPlotter(ROSWidget):

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

class ROSActionClientWidget(flx.Widget):

    def init(self, server_name, server_type):

        self.is_init = False
        self.server_name = server_name
        self.feedback_react = self.reaction(self._feedback_callback, "!root."+server_name.replace("/", "_")+"_feedback")
        self.result_react = self.reaction(self._result_callback, "!root."+server_name.replace("/", "_")+"_done")

        print("!root."+server_name.replace("/", "_")+"_result")

        with flx.GroupWidget(title=server_name, flex=1):
            with flx.FormLayout(flex=1):
                self.arguments = flx.LineEdit(title="Args", text="")
                self.feedback = flx.LineEdit(title="Feedback", text="")
                self.result = flx.LineEdit(title="Result", text="")
                self.send_goal = flx.Button(text="Send goal")
                flx.Widget(minsize=40)

        self.root.announce_action_client(self.server_name, server_type)

    @flx.reaction("send_goal.pointer_click")
    def _send_goal(self, *events):
        self.root.send_action_goal(self.server_name, self.arguments.text)
        self.feedback.set_text("Waiting...")
        self.result.set_text("Waiting...")

    def _feedback_callback(self, *events):

        print("Got new feedback!")
        for ev in events:
            self.feedback.set_text(str(ev.status))

    def _result_callback(self, *events):

        print("Got new result!")
        for ev in events:
            self.result.set_text(str(ev.status))
        self.feedback.set_text("")

