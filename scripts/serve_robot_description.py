#!/usr/bin/python3

import rospy
import re
import os
import shutil
import sys

import tornado.ioloop
import tornado.web

def relativize_description(desc):

    urls = re.findall('\"file:\/\/[^"]*\"', desc)

    if not os.path.exists("resources"):
        os.mkdir("resources")

    replacements = []

    p = re.compile('\"file:\/\/[^"]*\"')
    for m in p.finditer(desc):
        orig_path = os.path.abspath(m.group()[8:-1])
        new_path = os.path.join("resources", os.path.basename(orig_path))
        shutil.copy(orig_path, new_path)

        replacements.append(("file://" + orig_path, os.path.basename(orig_path)))

    for orig, new in replacements:
        desc = desc.replace(orig, new)

    return desc

class Handler(tornado.web.StaticFileHandler):

    def set_default_headers(self):
        print("setting headers!!!")
        self.set_header("Access-Control-Allow-Origin", "*")

def mkapp():
    application = tornado.web.Application([
        ('/(.*)', Handler, {'path': 'resources'}),
        ], debug=True)

    return application

def start_server(port):
    
    print('Starting server on port {}'.format(port))
    app = mkapp()
    loop = tornado.ioloop.IOLoop.current()
    try:
        app.listen(port)
        loop.start()
    finally:
        loop.stop()       # might be redundant, the loop has already stopped
        loop.close(True)  # needed to close all open sockets

def stop_loop():
    loop = tornado.ioloop.IOLoop.current()
    loop.stop()

if __name__ == "__main__":
    rospy.init_node("rospy_description_saver")
    robot_description = rospy.get_param('~robot_description', "/robot_description")
    flexxros_description = rospy.get_param('~flexxros_description', "/flexxros_description")
    desc = rospy.get_param(robot_description)
    desc = relativize_description(desc)
    rospy.set_param(flexxros_description, desc)

    rospy.on_shutdown(stop_loop)

    start_server(port=rospy.get_param('~resources_port', 9092))
