# flexxros
Ros GUI for webpages through flexx. Full documentation at https://nilsbore.github.io/flexxros-docs/

## Dependencies

```
pip3 install flexx rospkg
pip3 install tornado==5
```

## Running

```
rosrun flexxros example.py
```
This will present a webpage on `http://localhost:8097/`.

## Example application

The following script demonstrates a simple publish-subscribe application using flexxros.
It is identical to `example.py` in the scripts folder.

```
from flexx import flx, config
from flexxros import node
from flexxros.node import ROSNode, ROSWidget

class PublishSubscribeWidget(ROSWidget):

    def callback(self, msg):
        self.receive_message.set_text(msg.data)

    @flx.reaction("send_message.pointer_click")
    def _send_message(self, *events):
        self.publish("/test_topic", {"data": self.type_message.text})

    def init(self):
        with flx.FormLayout(flex=1, maxsize=400):
            self.type_message = flx.LineEdit(title="Message to send:", text="")
            self.send_message = flx.Button(text="Publish message")
            self.receive_message = flx.Label(title="Received message:", text="Waiting for message...")

        self.announce_publish("/test_topic", "std_msgs/String")
        self.subscribe("/test_topic", "std_msgs/String", self.callback)

class ExampleInterface(ROSNode):

    def init(self):
        self.main_widget = PublishSubscribeWidget()

config.hostname = "localhost"
config.port = 8097
node.init_and_spin("example_interface", ExampleInterface)
```
