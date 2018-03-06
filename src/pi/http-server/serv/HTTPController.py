#!/usr/bin/env python

import SimpleHTTPServer
import SocketServer
import rospy
import std_msgs
import signal
import sys

PORT = 8000

class HTTPController(SimpleHTTPServer.SimpleHTTPRequestHandler):
  def __init__(self,request,client_address,server):
    SimpleHTTPServer.SimpleHTTPRequestHandler.__init__(self,request,client_address,server)

  def do_POST(self):
    # with open(self.server.controller_args["log_file"],"w") as log:
    #   log.write("got here 2\n")
    print("got a POST")
    content_len = int(self.headers.getheader('content-length', 0))
    post_contents = self.rfile.read(content_len)
    print(post_contents)
    self.server.controller_args["publisher"].publish(post_contents)

class ControlServer(SocketServer.TCPServer):
  def __init__(self,server_address,RequestHandlerClass,controller_args={},bind_and_activate=True):
    SocketServer.TCPServer.__init__(self,server_address,RequestHandlerClass,bind_and_activate)
    self.controller_args = controller_args

def keyboard_interrupt():
  sys.exit(0);

def init():
  signal.signal(signal.SIGINT,keyboard_interrupt)
  rospy.init_node("server_controller")
  pub = rospy.Publisher("direct_control",std_msgs.msg.String,queue_size=10)
  handler = HTTPController
  httpd = ControlServer(("",PORT),handler,{"publisher":pub})

  print(httpd.server_address)

  try:
    httpd.serve_forever()
  except:
    print("***\nException raised\n***\nclosing server...")

if __name__ == "__main__":
  init()