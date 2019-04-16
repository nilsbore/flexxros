
from flexx import flx
import rospy
from .external.rospy_message_converter.src.rospy_message_converter import message_converter
#from external.rospy_message_converter.src.rospy_message_converter import json_message_converter
import importlib
import asyncio
import threading
import tornado.platform.asyncio as torasync
import dynamic_reconfigure.client
import actionlib

#class ROSRelay(flx.Component):
#
#    subscribers = flx.DictProp()
#    publishers = flx.DictProp()
#    reconfig_clients = flx.DictProp()
#
#    def init(self):
#        pass
#
#relay = ROSRelay()

#class ROSPublisher(flx.PyComponent):
class ROSPublisher:

    def __init__(self, topic, topic_type):
    #def init(self, topic, topic_type):

        self.topic_type = topic_type
        self.type = ROSNode.get_type_from_name(topic_type)
        self.pub = rospy.Publisher(topic, self.type, queue_size=10)

#class ROSSubscriber(flx.PyComponent):
class ROSSubscriber:

    #@relay.reaction('_pitch_feedback')
    def _callback(self, *events):
        topic = self.topic.replace("/", "_")
        for ev in events:
            self.parent.emit(topic, ev)

    def callback(self, msg):
        
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        topic = self.topic.replace("/", "_")
        #print("Got callback, emitting pitch")
        try:
            self.parent.emit(topic, msg_dict)
            #relay.emit(topic, msg_dict)
            #relay._pitch_feedback(msg_dict)
            #relay.topic_published(topic, msg_dict)
        except BufferError:
            pass
            #print("Caught a buffer error!")

    def __init__(self, parent, topic, topic_type):
    #def init(self, parent, topic, topic_type):

        self.parent = parent
        self.topic = topic
        self.sub = rospy.Subscriber(topic, ROSNode.get_type_from_name(topic_type), self.callback)
        #self.react = relay.reaction(self._callback, topic.replace("/", "_"))

class ROSDynReconfig:

    def set_config(self):
        if self.last_config is not None:
            self.client.update_configuration(self.last_config)

    def callback(self, config):

        print(config)
        self.last_config = config
        server_name = self.server_name.replace("/", "_")
        try:
            self.parent.emit(server_name, self.last_config)
            #relay.emit(server_name, self.last_config)
        except BufferError:
            print("Got buffer error in dyn reconfig")
        ##    pass

    def __init__(self, parent, server_name):
        self.last_config = None
        self.parent = parent
        self.server_name = server_name
        self.client = dynamic_reconfigure.client.Client(server_name, timeout=30, config_callback=self.callback)

class ROSActionClient:

    def __init__(self, parent, server_name, server_type):
        self.parent = parent
        self.server_name = server_name
        self.server_type = server_type
        self_type = ROSNode.get_type_from_name(server_type+"Action")
        self.client = actionlib.SimpleActionClient(server_name, self_type)

    def feedback_cb(self, status, msg):
        print("Got feedback: ", str(msg))
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        self.parent.emit(self.server_name.replace("/", "_")+"_feedback", msg_dict)

    def done_cb(self, status, msg):
        print("Got result: ", str(msg))
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        self.parent.emit(self.server_name.replace("/", "_")+"_done", msg_dict)

    def send_goal(self, msg_dict):
        #msg = message_converter.convert_dictionary_to_ros_message(self.type, msg_dict)
        #msg = json_message_converter.convert_json_to_ros_message(self.type, msg_dict)
        #goal_type = ROSNode.get_type_from_name(self.server_type+"Result") #"Goal")
        #print(goal_type)
        #gg = goal_type()
        #print(gg)
        msg = message_converter.convert_dictionary_to_ros_message(self.server_type+"Goal", {})
        print("Waiting for action server for 5s")
        self.client.wait_for_server(rospy.Duration.from_sec(5))
        print("Connected to action server")
        self.client.send_goal(msg, done_cb=self.done_cb, feedback_cb=self.feedback_cb, active_cb=None)

class ROSNode(flx.PyComponent):

    publishers = {} # flx.DictProp()
    action_clients = {}
    reconfig_clients = [] # flx.DictProp()
    subscribers = [] #flx.ListProp()
    #subscribers = flx.DictProp()

    def get_type_from_name(topic_type):

        # this code comes from the mongodb_store package
        parts = topic_type.split('/')
        cls_string = "%s.msg._%s.%s" % (parts[0], parts[1], parts[1])
        class_data = cls_string.split(".")
        module_path = ".".join(class_data[:-1])
        class_str = class_data[-1]
        module = importlib.import_module(module_path)

        return getattr(module, class_str)

    #@flx.reaction("relay.topic_published")
    #def topic_published(self, topic, value):
    #    self.emit(topic, value)

    @flx.action
    def subscribe(self, topic, topic_type):

        #print("Trying to subscribe to ", topic, " previous subscribers: ", relay.subscribers)

        # TODO: needs catching
        #with self:
        self.subscribers.append(ROSSubscriber(self, topic, topic_type))
        #if topic not in self.subscribers:
        #if topic not in relay.subscribers:
        #    with self:
        #        flx.mutate_dict(self.subscribers, {'mutation': 'insert', 'objects': {topic: ROSSubscriber(self, topic, topic_type)}})
        #    flx.mutate_dict(relay.subscribers, {'mutation': 'insert', 'objects': {topic: ""}})

    @flx.action
    def announce_publish(self, topic, topic_type):

        # this function actually seems superfluous
        # maybe just check in publish
        #with self:
        self.publishers[topic] = ROSPublisher(topic, topic_type)

    @flx.action
    def announce_action_client(self, server_name, server_type):

        # this function actually seems superfluous
        # maybe just check in publish
        #with self:
        self.action_clients[server_name] = ROSActionClient(self, server_name, server_type)

    @flx.action
    def announce_reconfig(self, server_name):
        self.reconfig_clients.append(ROSDynReconfig(self, server_name))
        #print("Reconfig clients: ", relay.reconfig_clients)
        #if server_name not in self.reconfig_clients:
        #    print("Adding new reconfig: ", server_name)
        #    flx.mutate_dict(self.reconfig_clients, {'mutation': 'insert', 'objects': {server_name: ROSDynReconfig(self, server_name)}})
        #    flx.mutate_dict(relay.reconfig_clients, {'mutation': 'insert', 'objects': {server_name: ""}})
        #else:
        #    print("Not add reconfig since already here: ", server_name)

    @flx.action
    def set_config(self, server_name, config):
        self.reconfig_clients[server_name].client.update_configuration(config)

    @flx.action
    def publish(self, topic, data):

        pub = self.publishers[topic]
        msg = message_converter.convert_dictionary_to_ros_message(pub.topic_type, data)
        pub.pub.publish(msg)

    @flx.action
    def send_action_goal(self, server_name, msg):
        self.action_clients[server_name].send_goal(msg)

    def init(self):
        pass

# Start server in its own thread
def start_flexx():
    flx.create_server(loop=asyncio.new_event_loop())
    flx.start()

def init_and_spin(node_name, app_type):

    rospy.init_node(node_name, anonymous=True)
    asyncio.set_event_loop_policy(torasync.AnyThreadEventLoopPolicy())

    # Create component in main thread
    app = flx.App(app_type)
    app.serve('')

    t = threading.Thread(target=start_flexx)
    t.start()

    rospy.spin()

    flx.stop()
    t.join()

class ROSTopicPlotter(flx.Widget):

    def init(self, topic, topic_type, key="data", yrange=(0, 100), nsamples=100):
        self.key = key
        self.nsamples = nsamples
        self.root.subscribe(topic, topic_type)
        #node.subscribe(topic, topic_type)
        self.start_time = time()
        #self.plot = self.mem_plot = flx.PlotWidget(flex=1, style='width: 500px; height: 320px;',
        self.plot = self.mem_plot = flx.PlotWidget(flex=1, style='width: 400px; height: 220px;',
                                                   xdata=[], yrange=yrange, ylabel=topic+"/"+key, xlabel="Time")
        self.react = self.reaction(self._callback, "!root."+topic.replace("/", "_"))

    def _callback(self, *events):

        #print("Got something from pitch feedback: ", events)

        #try:
        for ev in events:
            # Prepare plots
            times = list(self.plot.xdata)
            times.append(time() - self.start_time)
            times = times[-self.nsamples:]
            
            # cpu data
            values = list(self.plot.ydata)
            #values.append(ev.data)
            values.append(ev[self.key])
            values = values[-self.nsamples:]
            self.plot.set_data(times, values)

        #except BufferError:
        #    print("Got buffer error!")

class ROSDynReconfigWidget(flx.Widget):

    #is_init = flx.BoolProp(False)
        
    def init(self, server_name):

        self.is_init = False
        self.react = self.reaction(self._callback, "!root."+server_name.replace("/", "_"))

        with flx.GroupWidget(title=server_name):
            self.vbox = flx.FormLayout(flex=1)

        self.root.announce_reconfig(server_name)
        #node.announce_reconfig(server_name)
        #self.react.reconnect(0)

    @flx.action
    def add_children(self, ev):

        print("Got add children!")

        if self.is_init:
            #return
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
                #print(c, ev[c])
                if c in ["groups", "type", "source"]:
                    continue
                l = flx.LineEdit(title=c, text=str(ev[c]))
            flx.Button(text="Set")
            flx.Widget(minsize=60)

        #self._mutate_is_init(True)
        self.is_init = True

    def _callback(self, *events):

        print("Got new reconfig!")

        for ev in events:
            self.add_children(ev)

class ROSActionClientWidget(flx.Widget):

    CSS = """
        .flx-LineEdit {
            rows: 3;
        }
        """
        
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

