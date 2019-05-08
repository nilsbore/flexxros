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
    subscribers = {}

    @flx.action
    def subscribe(self, topic, topic_type):
        """
        Subscribe to a topic

        :param topic: the ROS topic name
        :param topic_type: the ROS message type in form of string, e.g. 'std_msgs/Float32'
        """

        if topic not in self.subscribers:
            try:
                self.subscribers[topic] = ROSSubscriber(self, topic, topic_type)
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

        try:
            self.publishers[topic] = ROSPublisher(topic, topic_type)
        except ImportError:
            print("Could not announce", topic, ", as", topic_type, "not recognized as type")

    @flx.action
    def announce_action_client(self, server_name, server_type):

        if server_name not in self.action_clients:
            try:
                self.action_clients[server_name] = ROSActionClient(server_name, server_type)
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
    def send_action_goal(self, server_name, msg):

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
    def subscribe(self, topic, topic_type, cb):
        """
        Subscribe to a topic

        :param topic: the ROS topic name
        :param topic_type: the ROS message type in form of string, e.g. 'std_msgs/Float32'
        :param cb: a callback handle, must be a method of a subclass of ROSWidget
        """
        self.root.subscribe(topic, topic_type)
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
