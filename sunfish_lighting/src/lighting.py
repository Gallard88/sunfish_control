#!/usr/bin/env python

from sunfish_lighting.srv import *
from sunfish_lighting.msg import *
from sunfish_ecu.srv import *
import rospy

class LightChannel(object):
  def __init__(self, c):
    self.channel = c
    self.duty = 0
    return

LightChannels = {}
light = LightChannel(10)
LightChannels["CH1"] = light
light = LightChannel(11)
LightChannels["CH2"] = light


class LightingServer(object):
  def __init__(self):
    rospy.wait_for_service('/sunfish/ecu/PWM')
    try:
      self.setPwm_service = rospy.ServiceProxy('/sunfish/ecu/PWM', setPWM)
    except rospy.ServiceException, e:
      rospy.loginfo("Service call failed: %s"% e )

    self.srv_OnOff = rospy.Service('/sunfish/lighting/on_off', OnOff, self.service_OnOff)
    self.pub = rospy.Publisher('/sunfish/lighting/status', Status, queue_size=10)
    self.sendStatus()
    return

  def rangeCheck(self, v):
    if v > 100:
      return 100
    if v < 0:
      return 0
    return v

  def service_OnOff(self, req):
    if req.Name in LightChannels:
      LightChannels[req.Name].duty = self.rangeCheck(req.Duty)
      duty    = LightChannels[req.Name].duty
      channel = LightChannels[req.Name].channel
      self.setPwm_service(duty, channel)
      self.sendStatus()
    return OnOffResponse()

  def sendStatus(self):
    stat = Status()
    stat.Name = []
    stat.Duty = []
    for key in LightChannels.keys():
      stat.Name.append(key)
      stat.Duty.append(LightChannels[key].duty)
    self.pub.publish(stat)
    return

  def updateECU(self):
    return

if __name__ == "__main__":
    rospy.init_node('loghting_server')
    light_server = LightingServer()
    for key in LightChannels.keys():
      oo = OnOff()
      oo.Name = key
      oo.Duty = 0
      light_server.service_OnOff(oo)

    rospy.loginfo("Lighting manager online")
    rospy.spin()

