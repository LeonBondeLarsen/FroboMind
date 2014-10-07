#!/usr/bin/env python


#https://docs.python.org/release/2.5.2/lib/socket-example.html

HOST="IcySpear"
PORT=9090

packet="{ \"op\": \"subscribe\", \"topic\": \"/turtle1/cmd_vel\" }"

print packet
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))
s.send(packet)
while 1:
	data = s.recv(1024)
	print data
