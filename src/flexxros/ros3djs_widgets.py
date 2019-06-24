#!/usr/bin/python3

from flexx import flx

base_url = "http://static.robotwebtools.org/"
safe_url = "https://static.robotwebtools.org/"
flx.assets.associate_asset(__name__, base_url + 'threejs/current/three.min.js')
flx.assets.associate_asset(__name__, base_url + 'EventEmitter2/current/eventemitter2.min.js')
flx.assets.associate_asset(__name__, base_url + 'roslibjs/current/roslib.min.js')
flx.assets.associate_asset(__name__, base_url + 'ros3djs/current/ros3d.min.js')
flx.assets.associate_asset(__name__, safe_url + 'threejs/current/STLLoader.js')
flx.assets.associate_asset(__name__, safe_url + 'ros3djs/current/ColladaLoader.js')

class RobotModelWidget(flx.Widget):
    """ A Robot model Rviz window based on ros3djs.
    """

    host_ip = flx.StringProp("127.0.0.1", settable=True, doc="""
                             The IP of the host computer.
                             """)

    base_link = flx.StringProp("/base_link", settable=True, doc="""
                               The base link frame of the robot.
                               """)

    rosbridge_port = flx.IntProp(9090, settable=True, doc="""
                                 The port of rosbridge websocket.
                                 """)

    resources_port = flx.IntProp(9091, settable=True, doc="""
                                 The port of web server serving .dae files for urdf.
                                 """)

    robot_description = flx.StringProp("/sam/simple_description", settable=True, doc="""
                                       The ros param with the robot description.
                                       """)

    def _create_dom(self):
        global document

        node = document.createElement('div')
        self.viewernode = document.createElement('div')
        self.viewernode.id = 'urdf'
        node.appendChild(self.viewernode)

        return node

    def _render_dom(self):
        self.viewer_init()
        return super()._render_dom()

    @flx.action
    def viewer_init(self):
        global window #, document

        if self.initialised:
            return

        self.ros = window.ROSLIB.Ros({
          'url' : 'ws://'+self.host_ip+':'+str(self.rosbridge_port)
        })

        # Create the main viewer.
        self.viewer = window.ROS3D.Viewer({
          'divID' : 'urdf',
          'width' : window.innerWidth,
          'height' : window.innerHeight,
          'antialias' : True,
          'background': '#002233'
        })

        # Add a grid.
        self.viewer.addObject(window.ROS3D.Grid({
          'color':'#0181c4'
        }))

        # Setup a client to listen to TFs.
        self.tfClient = window.ROSLIB.TFClient({
          'ros' : self.ros,
          'fixedFrame' : self.base_link, #"sam/base_link",
          'angularThres' : 0.01,
          'transThres' : 0.01,
          'rate' : 10.0
        })

        # Setup the URDF client.
        self.urdfClient = window.ROS3D.UrdfClient({
          'param' : self.robot_description,
          'path' : 'http://'+self.host_ip+':'+str(self.resources_port),
          'ros' : self.ros,
          'tfClient' : self.tfClient,
          'rootObject' : self.viewer.scene,
          'loader' : window.ROS3D.COLLADA_LOADER_2
        })

        self.initialised = True

    @flx.reaction('size')
    def __on_size(self, *events):
        if self.viewer:
            self.viewer.resize(window.innerWidth, window.innerHeight)


if __name__ == '__main__':
    flx.launch(RobotModelWidget, 'app')
    flx.run()
