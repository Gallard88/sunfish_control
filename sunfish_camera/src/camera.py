#!/usr/bin/env python

from sunfish_camera.srv import *
from sunfish_camera.msg import *
import rospy

class CameraServer(object):
  def __init__(self):
    self.stat = Status()
    self.stat.server_running = False
    self.stat.is_recording = False
    self.stat.recording_time = 0
    self.stat.port = 0
    self.stat.url = 'localhost'

    self.timer_ = rospy.Timer(rospy.Duration(1.0), self.timer_callback)
    self.srv_stream = rospy.Service('/sunfish/camera/stream', Stream, self.service_Stream)
    self.srv_record = rospy.Service('/sunfish/camera/record', Record, self.service_Record)
    self.pub = rospy.Publisher('/sunfish/camera/status', Status, queue_size=1)
    self.sendStatus()
    return

  def service_Stream(self, req):
    resp = False
    if req.action == req.START_STREAM:
      rospy.loginfo("Instructed to start streaming")
      resp = True
    elif req.action == req.STOP_STREAM:
      rospy.loginfo("Instructed to stopt streaming")
      resp = True
    return StreamResponse(resp)

  def service_Record(self, req):
    resp = False
    if req.action == req.START_RECORD:
      rospy.loginfo("Instructed to start recording")
      resp = True
    elif req.action == req.STOP_RECORD:
      rospy.loginfo("Instructed to stopt recording")
      resp = True
    return RecordResponse(resp)

  def timer_callback(self, time):
    # Here we send a status update
    if self.stat.is_recording == True:
      self.stat.recording_time = self.stat.recording_time + 1
    self.sendStatus()
    return

  def sendStatus(self):
    self.pub.publish(self.stat)
    return

if __name__ == "__main__":
    rospy.init_node('camera_server')
    cam_server = CameraServer()
    rospy.loginfo("Camera Server online")
    rospy.spin()
