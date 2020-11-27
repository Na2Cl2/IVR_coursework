#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import Float64MultiArray, Int16MultiArray, Float64

class joint_state:
    def __init__(self):
        # initialize the node named joint_processing
        rospy.init_node('joint_processing', anonymous=True)

        # initialize subscribers to receive the coordinates of the joints

        self.robot_x_z = message_filters.Subscriber("/robot/from_cam2", Int16MultiArray)
        #self.robot_y_z = message_filters.Subscriber("/robot/y_z", Int16MultiArray)
        self.robot_y_z = message_filters.Subscriber("/robot/from_cam1", Int16MultiArray)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.robot_x_z, self.robot_y_z],10, 0.1,allow_headerless=True)

        self.sync.registerCallback(self.callback)
        # initialize a publisher for joint angles
        self.joints_pub = rospy.Publisher("/joints_ang", Float64MultiArray, queue_size=10)
        # initialize a publisher for end effector position
        self.end_eff_pub = rospy.Publisher("/end_pos", Int16MultiArray, queue_size=10)

        self.joint2_pub = rospy.Publisher("/ja2", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/ja3", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/ja4", Float64, queue_size=10)
        
        self.target_x_pub = rospy.Publisher("/target_x_pos", Float64, queue_size=1)
        self.target_y_pub = rospy.Publisher("/target_y_pos", Float64, queue_size=1)
        self.target_z_pub = rospy.Publisher("/target_z_pos", Float64, queue_size=1)
        
        self.end_eff_x_pub = rospy.Publisher("/end_x_pos", Float64, queue_size=1)
        self.end_eff_y_pub = rospy.Publisher("/end_y_pos", Float64, queue_size=1)
        self.end_eff_z_pub = rospy.Publisher("/end_z_pos", Float64, queue_size=1)
        

        #[red_x,red_z,green_x,green_z,blue_x,blue_z,yellow_x,yellow_z,target_x,target_z]
        self.x_z = None
        #[red_y,red_z,green_y,green_z,blue_y,blue_z,yellow_y,yellow_z,target_y,target_z]
        self.y_z = None
    
    
    def get_joints(self,joints): 
    	print("get joints")
    	self.joints = np.array(joints.data)

    def detect_joint_angles(self,red,green,blue):
        new_z_1 = np.sqrt((green[0]-blue[0])**2+(green[2]-blue[2])**2)
        joint_2 = np.arctan2(np.abs(green[1]-blue[1]),np.abs(green[2]-blue[2]))
        if(green[1]>blue[1]):
            joint_2 = -joint_2
        new_z_2 = np.sqrt((green[1]-blue[1])**2+(green[2]-blue[2])**2)
        joint_3 = np.arctan2(np.abs(green[0]-blue[0]),new_z_2)
        if (green[0]<blue[0]):
            joint_3 = -joint_3
        green_to_blue = blue - green
        green_to_red = red - green
        joint_4 = np.pi - np.arccos(np.dot(green_to_blue, green_to_red) / (np.linalg.norm(green_to_blue) * np.linalg.norm(green_to_red)))
        if(red[0]<green[0]):
            joint_4 = -joint_4
        return np.array([joint_2,joint_3,joint_4])
    
    def forward_kinematics(self,joints):
    	a,b,c,d = joints
    	x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    	y = np.cos(a)*(np.sin(b)*np.cos(c)*(-3*np.cos(d)-3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    	z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    	end_effector = np.array([x,y,z])
    	return end_effector

    def callback(self,xz_pos,yz_pos):
        # cam2 [red_x,red_z,green_x,green_z,blue_x,blue_z,yellow_x,yellow_z,target_x,target_z]
        # cam1 [red_y,red_z,green_y,green_z,blue_y,blue_z,yellow_y,yellow_z,target_y,target_z]
        self.xz = xz_pos.data
        self.yz = yz_pos.data
        
        self.red_x = self.xz[0]
        self.red_y = self.yz[0]
        self.red_z_1 = self.xz[1]
        self.red_z_2 = self.yz[1]
        
        self.green_x = self.xz[2]
        self.green_y = self.yz[2]
        self.green_z_1 = self.xz[3]
        self.green_z_2 = self.yz[3]
        
        self.blue_x = self.xz[4]
        self.blue_y = self.yz[4]
        self.blue_z_1 = self.xz[5]
        self.blue_z_2 = self.yz[5]
        
        self.yellow_x = self.xz[6]
        self.yellow_y = self.yz[6]
        self.yellow_z_1 = self.xz[7]
        self.yellow_z_2 = self.yz[7]
        
        self.target_x = self.xz[8]
        self.target_y = self.yz[8]
        self.target_z_1 = self.xz[9]
        self.target_z_2 = self.yz[9]
        
        if(self.red_z_2 == 0):self.red_z = self.red_z_1
        else: self.red_z = self.red_z_2
        if (self.green_z_2 == 0):self.green_z = self.green_z_1
        else:self.green_z = self.green_z_2
        if (self.blue_z_2 == 0):self.blue_z = self.blue_z_1
        else:self.blue_z = self.blue_z_2
        if (self.yellow_z_2 == 0):self.yellow_z = self.yellow_z_1
        else:self.yellow_z = self.yellow_z_2
        if (self.target_z_2 == 0): self.target_z =self.target_z_1
        else:self.target_z = self.target_z_2
        
        
        self.target_x_pub.publish((self.target_x-self.yellow_x)/26)
        self.target_y_pub.publish((self.target_y-self.yellow_y)/26)
        self.target_z_pub.publish(-(self.target_z-self.yellow_z)/26)
        
        self.red = np.array([self.red_x-392,self.red_y-392,-self.red_z+529])
        self.green = np.array([self.green_x-392,self.green_y-392,-self.green_z+529])
        self.blue = np.array([self.blue_x-392,self.blue_y-392,-self.blue_z+529])
        
        self.target = np.array([-(392-self.target_x)*0.038,-(392-self.target_y)*0.038,(529-self.target_z)*0.038])
        
        
        self.joint_angles = Float64MultiArray()
        self.joint_angles.data = self.detect_joint_angles(self.red,self.green,self.blue)
        
        joint2 = self.joint_angles.data[0]
        joint3 = self.joint_angles.data[1]
        joint4 = self.joint_angles.data[2]
        self.target_pos = Int16MultiArray()
        self.target_pos.data = self.target
        self.end_eff_pos = Int16MultiArray()
        self.red = np.array([-(self.yellow_x-self.red_x)*0.038 ,-(self.yellow_y-self.red_y)*0.038,(self.yellow_z-self.red_z)*0.038])
        self.end_eff_pos.data = self.red
        # Publish the joint angles
        
        self.joints_pub.publish(self.joint_angles)
        #self.target_pub.publish(self.target_pos)
        self.end_eff_pub.publish(self.end_eff_pos)
        
        self.end_eff_x_pub.publish(self.red[0])
        self.end_eff_y_pub.publish(self.red[1])
        self.end_eff_z_pub.publish(self.red[2])

        self.joint2_pub.publish(joint2)
        self.joint3_pub.publish(joint3)
        self.joint4_pub.publish(joint4)
        
        print(self.end_eff_pos)

  
# call the class
def main(args):
  ic = joint_state()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
