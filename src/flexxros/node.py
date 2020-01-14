from flexx import flx
import rospy
from .external.rospy_message_converter.src.rospy_message_converter import message_converter
import asyncio
import threading
import tornado.platform.asyncio as torasync
from flexxros.internal import ROSPublisher, ROSSubscriber, ROSActionClient, ROSServiceClient, ROSDynReconfig

class ROSNode(flx.PyComponent):
    """
    A PyComponent interface that the root node in the interface must always inherit from
    """

    publishers = {}
    action_clients = {}
    reconfig_clients = []
    subscribers = {}
    service_clients = {}

    @flx.action
    def subscribe(self, topic, topic_type, hz):
        """
        Subscribe to a topic

        :param topic: the ROS topic name
        :param topic_type: the ROS message type in form of string, e.g. 'std_msgs/Float32'
        """

        if topic not in self.subscribers:
            try:
                self.subscribers[topic] = ROSSubscriber(self, topic, topic_type, hz)
            except ImportError:
                print("Could not subscribe to", topic, ", as", topic_type, "not recognized as type")
        else:
            self.subscribers[topic].add_parent(self)

    @flx.action
    def announce_publish(self, topic, topic_type):
        """
        Announce publisher, must be called before publish

        :param topic: the ROS topic name
        :param topic_type: the ROS message type in form of string, e.g. 'std_msgs/Float32'
        """
        if topic in self.publishers:
            return

        try:
            self.publishers[topic] = ROSPublisher(topic, topic_type)
        except ImportError:
            print("Could not announce", topic, ", as", topic_type, "not recognized as type")

    @flx.action
    def announce_action_client(self, server_name, server_type):
        """
        Announce ROS action client, must be called before publish

        :param server_name: the ROS action server name
        :param server_type: the ROS action server type in form of string, e.g. 'flexxros/Test'
        """
        if server_name in self.action_clients:
            self.emit(server_name.replace("/", "_")+"_prototype", self.action_clients[server_name].goal_prototype)
            return

        try:
            self.action_clients[server_name] = ROSActionClient(server_name, server_type)
            self.emit(server_name.replace("/", "_")+"_prototype", self.action_clients[server_name].goal_prototype)
        except ImportError:
            print("Could not announce client", server_name, ", as", server_type, "not recognized as type")

    @flx.action
    def announce_reconfig(self, server_name):
        """
        Experimental
        """
        self.reconfig_clients.append(ROSDynReconfig(self, server_name))

    @flx.action
    def set_config(self, server_name, config):
        """
        Experimental
        """
        self.reconfig_clients[server_name].client.update_configuration(config)

    @flx.action
    def publish(self, topic, data):
        """
        Publish a message to a topic, must call announce_publish before this

        :param topic: the ROS topic name
        :param data: a dictionary version of message, e.g. {data: 0.32}
        """

        try:
            pub = self.publishers[topic]
        except KeyError:
            print("Could not publish", topic, "as it is not announced, please call announce_publish before")
            return
        msg = message_converter.convert_dictionary_to_ros_message(pub.topic_type, data)
        pub.pub.publish(msg)

    @flx.action
    def call_service(self, server_name, server_type, req):
        """
        Call a ROS service

        :param server_name: the ROS service name
        :param server_type: the ROS service type, e.g. std_srvs/SetBool
        :param req: a dictionary version of message, e.g. {data: 0.32}
        """
        if server_name not in self.service_clients:
            try:
                self.service_clients[server_name] = ROSServiceClient(server_name, server_type)
            except ImportError:
                print("Could not get instance of", server_name, ", as", server_type, "not recognized as type")

        srv = self.service_clients[server_name]
        resp = srv.call_service(req)
        self.emit(server_name.replace("/", "_")+"_response", resp)

    @flx.action
    def send_action_goal(self, server_name, msg):
        """
        Send a ROS action goal, need to call announce_action_client before

        :param server_name: the ROS action server name
        :param msg: a dictionary version of the action goal message, e.g. {data: 0.32}
        """

        try:
            self.action_clients[server_name].send_goal(msg, self)
        except KeyError:
            print("Could not call", server_name, "as it is not announced, please call announce_action_client before")

    def init(self):
        pass

class ROSWidget(flx.Widget):
    """
    flx.Widget subclass that a widget can inherit from for easy access to ros functionality
    """

    def init(self):
        pass

    @flx.action
    def announce_action_client(self, topic, topic_type):
        """
        Announce ROS action client, must be called before publish

        :param topic: the ROS action client name
        :param topic_type: the ROS action server type in form of string, e.g. 'flexxros/Test'
        """

        self.root.announce_action_client(topic, topic_type)

    @flx.action
    def call_service(self, server_name, server_type, req, cb):
        """
        Call a ROS service

        :param server_name: the ROS service name
        :param server_type: the ROS service type, e.g. std_srvs/SetBool
        :param req: a dictionary version of message, e.g. {data: 0.32}
        :param cb: a callback handle, must be a method of a subclass of ROSWidget that takes result as arguments
        """

        self.reaction(cb, "!root."+server_name.replace("/", "_")+"_response")
        self.root.call_service(server_name, server_type, req)

    @flx.action
    def send_action_goal(self, topic, goal, feedback_cb, done_cb):
        """
        Send a ROS action goal, need to call announce_action_client before

        :param topic: the ROS action server name
        :param goal: a dictionary version of the action goal message, e.g. {data: 0.32}
        :param feedback_cb: a callback handle, must be a method of a subclass of ROSWidget that takes status, and feedback as arguments
        :param done_cb: a callback handle, must be a method of a subclass of ROSWidget that takes result as argument
        """

        self.root.send_action_goal(topic, goal)
        self.reaction(feedback_cb, "!root."+topic.replace("/", "_")+"_feedback")
        self.reaction(done_cb, "!root."+topic.replace("/", "_")+"_done")

    @flx.action
    def announce_publish(self, topic, topic_type):
        """
        Announce publisher, must be called before publish

        :param topic: the ROS topic name
        :param topic_type: the ROS message type in form of string, e.g. 'std_msgs/Float32'
        """

        self.root.announce_publish(topic, topic_type)

    @flx.action
    def publish(self, topic, data):
        """
        Publish a message to a topic, must call announce_publish before this

        :param topic: the ROS topic name
        :param data: a dictionary version of message, e.g. {data: 0.32}
        """

        self.root.publish(topic, data)

    @flx.action
    def subscribe(self, topic, topic_type, cb, hz=-1):
        """
        Subscribe to a topic

        :param topic: the ROS topic name
        :param topic_type: the ROS message type in form of string, e.g. 'std_msgs/Float32'
        :param cb: a callback handle, must be a method of a subclass of ROSWidget
        """
        self.root.subscribe(topic, topic_type, hz)
        self.reaction(cb, "!root."+topic.replace("/", "_"))

# Start server in its own thread
def start_flexx():
    """
    Used by init_and_spin to launch flexx
    """

    flx.create_server(loop=asyncio.new_event_loop())
    flx.start()

def spin(app_type):
    """
    The main way of starting your flexxros app 

    :param app_type: the root widget, must inherit from ROSNode
    """

    asyncio.set_event_loop_policy(torasync.AnyThreadEventLoopPolicy())

    # Create component in main thread
    app = flx.App(app_type)
    app.serve('')

    t = threading.Thread(target=start_flexx)
    t.start()

    rospy.spin()

    flx.stop()
    t.join()

def init_and_spin(node_name, app_type):
    """
    The main way of starting your flexxros app 

    :param node_name: the ROS node name
    :param app_type: the root widget, must inherit from ROSNode
    """

    rospy.init_node(node_name, anonymous=True)
    spin(app_type)
