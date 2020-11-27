#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('rotates', anonymous=True)

    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    
    self.joints_sub = rospy.Subscriber("/joints_ang",Float64MultiArray,self.get_joints)

    self.joint1pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=1)
    self.joint2pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=1)
    self.joint3pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=1)
    self.joint4pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=1)
    
    
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
    


  def get_joints(self,joints): 
      print("get joints")
      self.joints = np.array(joints.data)
  
  #Recieve data, process it, and publish
  def callback1(self,data):

    self.t = rospy.get_time()
       
    joint1 = 0
    joint2 = (np.pi / 2) * np.sin((np.pi / 15) * self.t)
    joint3 = (np.pi / 2) * np.sin((np.pi / 18) * self.t)
    joint4 = (np.pi / 2) * np.sin((np.pi / 20) * self.t)

    
    # Recieve the image
    try:
      #self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.joint1pub.publish(joint1)
      self.joint2pub.publish(joint2)
      self.joint3pub.publish(joint3)
      self.joint4pub.publish(joint4)
    except CvBridgeError as e:
      print(e)
    
    #print(self.joint2pub.data)

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

