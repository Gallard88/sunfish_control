#!/usr/bin/env python

from sunfish_lighting.srv import *
from sunfish_lighting.msg import *
from sunfish_ecu.srv import *
import rospy

class LightingServer(object):
  def __init__(self):
    self.channels = {}

    param_name = '/sunfish/lighting/channels'
    if rospy.has_param(param_name):
      rospy.logdebug("Loading Light Channel settings")
      items = rospy.get_param(param_name)
      for key in items:
        self.channels[key] = {}
        self.channels[key]['CH']   = items[key]
        self.channels[key]['Duty'] = 0
        rospy.logdebug("Loaded '%s' (%02d)" % (key, items[key]))

    try:
      rospy.wait_for_service('/sunfish/ecu/PWM', timeout=1)
      self.setPwm_service = rospy.ServiceProxy('/sunfish/ecu/PWM', setPWM)
    except rospy.ServiceException, e:
      rospy.logfatal("Service call failed: %s"% e )
      exit(-1)
    except rospy.ROSException, re:
      rospy.logfatal("Service call timed out %s" % re )
      exit(-1)

    self.srv_OnOff = rospy.Service('/sunfish/lighting/on_off', OnOff, self.service_OnOff)
    self.pub = rospy.Publisher('/sunfish/lighting/status', Status, queue_size=10)

    for key in self.channels:
      oo = OnOff()
      oo.Name = key
      oo.Duty = 0
      self.service_OnOff(oo)


    self.sendStatus()
    return

  def rangeCheck(self, v):
    if v > 100:
      return 100
    if v < 0:
      return 0
    return v

  def service_OnOff(self, req):
    if req.Name in self.channels:
      self.channels[req.Name]['Duty'] = self.rangeCheck(req.Duty)
      duty    = self.channels[req.Name]['Duty']
      channel = self.channels[req.Name]['CH']
      self.setPwm_service(duty, channel)
      self.sendStatus()
    return OnOffResponse()

  def sendStatus(self):
    stat = Status()
    stat.Name = []
    stat.Duty = []
    for key in self.channels:
      stat.Name.append(key)
      stat.Duty.append(self.channels[key]['Duty'])
    self.pub.publish(stat)
    return

  def updateECU(self):
    return

if __name__ == "__main__":
    rospy.init_node('lighting_server')
    rospy.loginfo("Lighting manager online")

    light_server = LightingServer()
    rospy.loginfo("Server ready")
    rospy.spin()

