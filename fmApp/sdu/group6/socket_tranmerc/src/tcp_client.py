#!/usr/bin/env python
from rospy import init_node, get_param, loginfo, logerr, is_shutdown
#from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from signal import signal, SIGINT, SIG_DFL

import asyncore, socket, sys, time, rospy

from msgs.msg import gpgga_tranmerc

class TCPBridgeClient(asyncore.dispatcher):
	def __init__(self,host,port=9090, order=7):
		print self.__class__,"__INIT__"
		asyncore.dispatcher.__init__(self)
		self.order = order

		self.host, self.port = host, int(port)
		self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
		self.connect( (self.host, self.port) )
		self.buffer=""
		self.gps_pub = rospy.Publisher("/fmInformation/gpgga_tranmerc", gpgga_tranmerc, queue_size=10)
	
	def handle_connect(self):
		print 'Connected to {}:{}.'.format(self.host,str(self.port))

	def handle_close(self):
		print 'Connection closed.'
		self.close()

	def handdle_error(self):
		print self.__class__,"handle_error"

	def handle_read(self):

              data = self.recv(1024)
              if not data:
                  return

              datalist = data.split(',')
              timestamp = datalist[2]
              for i in range( (len(datalist) - 3 )/3):
                  if str(self.order) == datalist[ 3 + i*3]:
                      x = float(datalist[4 + i*3])
                      y = float(datalist[5 + i*3])
                      
                      
              """ create and pubish tranmerc """
              gps_message = gpgga_tranmerc()
              gps_message.time = timestamp
              gps_message.northing = y/100
              gps_message.easting = x/100
              self.gps_pub.publish(gps_message) 

         
 

	def writable(self):
		return len(self.buffer)>0

	def handle_write(self):
		sent = self.send(self.buffer)
		self.buffer = self.buffer[sent:]

# list of possible parameters ( with internal default values <-- get overwritten from parameter server and commandline)
port = 9090                             # integer (portnumber)
host = '192.168.1.50'                      # hostname / IP as string

if __name__ == "__main__":
    init_node("marker_locator_tcp_client" )
    order = rospy.get_param("~order", 7)
    print "Order is set to: " + str( order )

    signal(SIGINT, SIG_DFL)

    client = TCPBridgeClient(host,port,order)
    asyncore.loop()
