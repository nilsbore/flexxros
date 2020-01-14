import rospy
from .external.rospy_message_converter.src.rospy_message_converter import message_converter
import importlib
import actionlib
import dynamic_reconfigure.client
import json
from functools import partial

def get_type_from_name(topic_type):

    # this code comes from the mongodb_store package
    parts = topic_type.split('/')
    cls_string = "%s.msg._%s.%s" % (parts[0], parts[1], parts[1])
    class_data = cls_string.split(".")
    module_path = ".".join(class_data[:-1])
    class_str = class_data[-1]
    module = importlib.import_module(module_path)

    return getattr(module, class_str)

def get_service_type_from_name(topic_type):

    # this code comes from the mongodb_store package
    parts = topic_type.split('/')
    cls_string = "%s.srv._%s.%s" % (parts[0], parts[1], parts[1])
    class_data = cls_string.split(".")
    module_path = ".".join(class_data[:-1])
    class_str = class_data[-1]
    module = importlib.import_module(module_path)

    return getattr(module, class_str)

class ROSPublisher:
    """
    Internal class to handle publishers
    """

    def __init__(self, topic, topic_type):

        self.topic_type = topic_type
        self.type = get_type_from_name(topic_type)
        self.pub = rospy.Publisher(topic, self.type, queue_size=10)

class ROSSubscriber:
    """
    Internal class to handle subscribers
    """

    #def _callback(self, *events):
    #    topic = self.topic.replace("/", "_")
    #    for ev in events:
    #        self.parent.emit(topic, ev)

    def callback(self, msg):

        msg_time = rospy.get_time()
        if self.interval != -1 and self.last_emitted > 0. and msg_time - self.last_emitted < self.interval:
            return
        
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        topic = self.topic.replace("/", "_")
        try:
            for parent in self.parents:
                parent.emit(topic, msg_dict)
        except BufferError:
            pass
            #print("Caught a buffer error!")

        self.last_emitted = msg_time

    def add_parent(self, parent):

        if parent not in self.parents:
            self.parents.append(parent)
        else:
            print("Parent already in subscribers, not adding!")

    def __init__(self, parent, topic, topic_type, hz):

        self.parents = [parent]
        self.topic = topic
        self.interval = 1./hz if hz > 0 else -1
        self.last_emitted = 0.
        self.sub = rospy.Subscriber(topic, get_type_from_name(topic_type), self.callback)

class ROSDynReconfig:
    """
    Internal class to handle dynamic reconfigure
    """

    def set_config(self):
        if self.last_config is not None:
            self.client.update_configuration(self.last_config)

    def callback(self, config):

        print(config)
        self.last_config = config
        server_name = self.server_name.replace("/", "_")
        try:
            self.parent.emit(server_name, self.last_config)
        except BufferError:
            print("Got buffer error in dyn reconfig")

    def __init__(self, parent, server_name):
        self.last_config = None
        self.parent = parent
        self.server_name = server_name
        self.client = dynamic_reconfigure.client.Client(server_name, timeout=30, config_callback=self.callback)

class ROSActionClient:
    """
    Internal class to handle action clients
    """

    def __init__(self, server_name, server_type):
        self.server_name = server_name
        self.server_type = server_type
        self_type = get_type_from_name(server_type+"Action")
        self.goal_prototype = {"data": json.dumps(message_converter.convert_ros_message_to_dictionary(get_type_from_name(server_type+"Goal")()))}
        self.client = actionlib.SimpleActionClient(server_name, self_type)

    def feedback_cb(self, parent, msg):
        print("Got feedback: ", str(msg))
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        parent.emit(self.server_name.replace("/", "_")+"_feedback", msg_dict)

    def done_cb(self, parent, status, msg):
        print("Got result: ", str(msg))
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        parent.emit(self.server_name.replace("/", "_")+"_done", msg_dict)

    def send_goal(self, msg_dict_str, parent):
        try:
            msg_dict = json.loads(msg_dict_str)
        except json.JSONDecodeError:
            print("Could not decode %s, sending empty goal...", msg_dict_str)
            msg_dict = {}

        msg = message_converter.convert_dictionary_to_ros_message(self.server_type+"Goal", msg_dict)
        print("Waiting for action server for 5s")
        if not self.client.wait_for_server(rospy.Duration.from_sec(5)):
            print("Could not connect to server, please start", self.server_name)
            return
        print("Connected to action server")
        self.client.send_goal(msg, done_cb=partial(self.done_cb, parent), feedback_cb=partial(self.feedback_cb, parent), active_cb=None)

class ROSServiceClient:
    """
    Internal class to handle service clients
    """

    def __init__(self, server_name, server_type):
        self.server_name = server_name
        self.server_type = server_type
        self.self_type = get_service_type_from_name(server_type)

    def call_service(self, req_dict):

        req = message_converter.convert_dictionary_to_ros_message(self.server_type, req_dict, kind='request')
        rospy.wait_for_service(self.server_name)
        try:
            client = rospy.ServiceProxy(self.server_name, self.self_type)
            resp = client(req)
        except rospy.ServiceException(e):
            print("Service call failed: %s", e)

        resp_dict = message_converter.convert_ros_message_to_dictionary(resp)
        return resp_dict
