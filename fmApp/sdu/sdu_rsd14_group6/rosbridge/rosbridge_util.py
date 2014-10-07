#!/usr/bin/env python

import json

class RosBridgeMsgFactory(object):
	@staticmethod
	def Advertise(topic, messageType):
		return json.dumps({'op':'advertise','topic':topic,'type':messageType})
	@staticmethod
	def Unadvertise(topic):
		return json.dumps({'op':'unadvertise','topic':topic})
	@staticmethod
	def Publish(topic,msg):
		return json.dumps({'op':'publish','topic':topic,'msg':msg})
	@staticmethod
	def Subscribe(topic):
		return json.dumps({'op':'subscribe','topic':topic})
	@staticmethod
	def Unsubscribe(topic):
		return json.dumps({'op':'unsubscribe','topic':topic})
