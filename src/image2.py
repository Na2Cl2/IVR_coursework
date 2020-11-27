#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize subscriber to receive the y z coordinates
    self.cam2_pub = rospy.Publisher("/robot/from_cam2", Int16MultiArray, queue_size=10)


  def detect_red(self,image):
    mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      return np.array([0,0])
    cx = int(M['m10'] / M['m00'])
    cz = int(M['m01'] / M['m00'])
    return np.array([cx, cz])

  def detect_green(self,image):
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      # We can only obtain the y coordiante now
      return np.array([0, 0])
    cx = int(M['m10'] / M['m00'])
    cz = int(M['m01'] / M['m00'])
    return np.array([cx, cz])

  def detect_blue(self,image):
    mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      return np.array([0, 0])
    cx = int(M['m10'] / M['m00'])
    cz = int(M['m01'] / M['m00'])
    return np.array([cx, cz])

  def detect_yellow(self,image):
    mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    if M['m00'] == 0:
      return np.array([0, 0])
    cx = int(M['m10'] / M['m00'])
    cz = int(M['m01'] / M['m00'])
    return np.array([cx, cz])
        
  def detect_target(self,image):
    mask = cv2.inRange(image, (5, 50, 50), (25, 255, 255))
    ret, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    cnt, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if(len(cnt)>=2):
      c = max(cnt,key=len)
      (cx, cz), radius = cv2.minEnclosingCircle(c)
      return np.array([int(cx), int(cz)])
    else:
      if(len(cnt)<1): return [0,0]
      if(len(cnt[0])<15):
        return np.array([0, 0])
      else:
        kernel = np.ones((3, 3), np.uint8)
        ero = cv2.erode(mask, kernel, iterations=1)
        ret, thresh = cv2.threshold(ero, 127, 255, cv2.THRESH_BINARY)
        cnt, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        while(len(cnt)!=2):
          ero = cv2.erode(ero, kernel, iterations=1)
          ret, thresh = cv2.threshold(ero, 127, 255, cv2.THRESH_BINARY)
          cnt, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        c = max(cnt, key=len)
        (cx, cz), radius = cv2.minEnclosingCircle(c)
        return np.array([int(cx), int(cz)])


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy_2.png', self.cv_image2)

    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

    red = self.detect_red(self.cv_image2)
    green = self.detect_green(self.cv_image2)
    blue = self.detect_blue(self.cv_image2)
    target = self.detect_target(self.cv_image2)
    yellow = self.detect_yellow(self.cv_image2)

    self.pub = Int16MultiArray()
    self.pub.data = np.array([red[0], red[1], green[0],green[1], 
    				blue[0], blue[1],yellow[0], yellow[1],
                              target[0],target[1]])
    # Publish the x and z coordinates
    self.cam2_pub.publish(self.pub)
    #print(self.pub)





# call the class
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


