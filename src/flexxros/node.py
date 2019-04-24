from flexx import flx
import rospy
from .external.rospy_message_converter.src.rospy_message_converter import message_converter
import asyncio
import threading
import tornado.platform.asyncio as torasync
from flexxros.internal import ROSPublisher, ROSSubscriber, ROSActionClient, ROSDynReconfig

class ROSNode(flx.PyComponent):
    """
    A PyComponent interface that the root node in the interface must always inherit from
    """

    publishers = {}
    action_clients = {}
    reconfig_clients = []
    subscribers = []

    @flx.action
    def subscribe(self, topic, topic_type):

        # TODO: needs catching
        try:
            self.subscribers.append(ROSSubscriber(self, topic, topic_type))
        except ImportError:
            print("Could not subscribe to", topic, ", as", topic_type, "not recognized as type")

    @flx.action
    def announce_publish(self, topic, topic_type):

        try:
            self.publishers[topic] = ROSPublisher(topic, topic_type)
        except ImportError:
            print("Could not announce", topic, ", as", topic_type, "not recognized as type")

    @flx.action
    def announce_action_client(self, server_name, server_type):

        try:
            self.action_clients[server_name] = ROSActionClient(self, server_name, server_type)
        except ImportError:
            print("Could not announce client", server_name, ", as", server_type, "not recognized as type")

    @flx.action
    def announce_reconfig(self, server_name):
        self.reconfig_clients.append(ROSDynReconfig(self, server_name))

    @flx.action
    def set_config(self, server_name, config):
        self.reconfig_clients[server_name].client.update_configuration(config)

    @flx.action
    def publish(self, topic, data):

        try:
            pub = self.publishers[topic]
        except KeyError:
            print("Could not publish", topic, "as it is not announced, please call announce_publish before")
            return
        msg = message_converter.convert_dictionary_to_ros_message(pub.topic_type, data)
        pub.pub.publish(msg)

    @flx.action
    def send_action_goal(self, server_name, msg):

        try:
            self.action_clients[server_name].send_goal(msg)
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

    def announce_publish(self, topic, topic_type):
        self.root.announce_publish(topic, topic_type)

    def publish(self, topic, data):
        self.root.publish(topic, data)

    def subscribe(self, topic, topic_type, cb):
        self.root.subscribe(topic, topic_type)
        self.reaction(cb, "!root."+topic.replace("/", "_"))

# Start server in its own thread
def start_flexx():
    flx.create_server(loop=asyncio.new_event_loop())
    flx.start()

def init_and_spin(node_name, app_type):
    """
    The main way of starting your flexxros app, takes the root 

    :param node_name: the ROS node name
    :param app_type: the root widget, must inherit from ROSNode
    """

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
