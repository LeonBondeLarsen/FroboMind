#!/usr/bin/env python
#https://docs.python.org/release/2.5.2/lib/socket-example.html

import socket, time

from rosbridge_util import RosBridgeMsgFactory as MF

REMOTE=("IcySpear",9090)

data = [({'x':2.0,'y':0.0,'z':0.0},{'x':0.0,'y':0.0,'z':0.0}),({'x':0.0,'y':0.0,'z':0.0},{'x':0.0,'y':0.0,'z':2.0})]

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(REMOTE)
s.send(MF.Advertise("/turtle1/cmd_vel","geometry_msgs/Twist"))
i = 0
while 1:
	packet = MF.Publish("/turtle1/cmd_vel",{'linear':data[i%len(data)][0],'angular':data[i%len(data)][1]})
	s.send(packet)
	print packet
	i+=1
	time.sleep(1)
