#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        rate = rospy.Rate(30) # 30hz
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        
        self.end_effector_posex = rospy.Publisher("/end_eff_x", Float64, queue_size=10)
        self.end_effector_posey = rospy.Publisher("/end_eff_y", Float64, queue_size=10)
        self.end_effector_posez = rospy.Publisher("/end_eff_z", Float64, queue_size=10)
        
        
        self.current_joint = np.array([0.0,0.0,0.0,0.0])
        self.joint_sub = rospy.Subscriber("/robot/joint_states",JointState,self.callback)
        
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        self.bridge = CvBridge()
        #self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.cam1)
        #self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.cam2)
        self.time_trajectory = rospy.get_time()
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.error = np.array([0.0, 0.0,0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0,0.0], dtype='float64')
        self.Y = np.array([400, 397, 530])
        self.orange = [0,0,0]
        self.target1_sub = rospy.Subscriber("/target/joint_states",JointState,self.target1_callback)
        
        self.pixelToMeter = 0.0388
        while not rospy.is_shutdown():
            rate.sleep()

    def target1_callback(self,orangePos):
    	print(orangePos.position)
    	self.orange= np.array(orangePos.position)
        
    
    def callback(self,data):
        
        self.current_joint=np.array([x for x in data.position])

        
        endpostion=self.fk(self.current_joint)
        v1 = endpostion-self.orange 
        self.v = v1/(np.linalg.norm(v1, 2, axis=0))
        
        q=self.control_closed(self.current_joint)
        self.robot_joint1_pub.publish(q[0])
        self.robot_joint2_pub.publish(q[1])
        self.robot_joint3_pub.publish(q[2])
        self.robot_joint4_pub.publish(q[3])
    #print(self.current_joint)
    
    
    def control_closed(self,current_joint):
        # P gain
        K_p = np.array([[20, 0,0], [0,20, 0],[0,0,20]])
        # D gain
        K_d = np.array([[0.4, 0,0], [0,0.4, 0],[0,0,0.14]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time
        
        
        # robot end-effector position
        pos = self.fk(current_joint)
        self.end_effector_posex.publish(pos[0])
        self.end_effector_posey.publish(pos[1])
        self.end_effector_posez.publish(pos[2])
        #print(pos)
        # desired trajectory

        pos_d =self.orange
        
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / (dt+1e-11) # avoid 0 division
        # estimate error
        self.error = pos_d - pos
        #print(self.error)
        
        J=self.jacobian(current_joint)
        J_inv = np.linalg.pinv(self.jacobian(current_joint))  #the psudeo inverse of Jacobian
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p,self.error.transpose())))  #control input, angular velocity of joints
        q0 = np.dot(J.transpose(),self.v)
        I = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        q_d = (dt * dq_d)  # control input, angular position of joints
        return current_joint+q_d+7e+10*np.dot(I-np.dot(J_inv,J),q0)
    
    def fk(self,joints):
        ja1,ja2,ja3,ja4 = joints
        end_effector = np.array([3*np.sin(ja1)*np.sin(ja2)*np.cos(ja3)*np.cos(ja4)
                                  + 3.5*np.sin(ja1)*np.sin(ja2)*np.cos(ja3)
                                  + 3*np.cos(ja1)*np.cos(ja4)*np.sin(ja3)
                                  + 3.5*np.cos(ja1)*np.sin(ja3)
                                  + 3*np.sin(ja1)*np.cos(ja2)*np.sin(ja4),

                                  -3*np.cos(ja1)*np.sin(ja2)*np.cos(ja3)*np.cos(ja4)
                                  - 3.5*np.cos(ja1)*np.sin(ja2)*np.cos(ja3)
                                  + 3*np.sin(ja1)*np.cos(ja4)*np.sin(ja3)
                                  + 3.5*np.sin(ja1)*np.sin(ja3)
                                  - 3*np.cos(ja1)*np.cos(ja2)*np.sin(ja4),

                                  3*np.cos(ja2)*np.cos(ja3)*np.cos(ja4)
                                  +3.5*np.cos(ja2)*np.cos(ja3)
                                  -3*np.sin(ja2)*np.sin(ja4)+2.5
                                  ])
        return end_effector
    
    def jacobian(self,joints):
        ja1,ja2,ja3,ja4 = joints
        jacobian=np.array([[3*(np.cos(ja1)*np.sin(ja2)*np.cos(ja3)*np.cos(ja4)
                            -np.sin(ja1)*np.cos(ja4)*np.sin(ja3)
                            +np.cos(ja1)*np.cos(ja2)*np.sin(ja4))
                            +3.5*(np.cos(ja1)*np.sin(ja2)*np.cos(ja3)
                            -np.sin(ja1)*np.sin(ja3)),

                            3*(np.sin(ja1)*np.cos(ja2)*np.cos(ja3)*np.cos(ja4)
                            -np.sin(ja1)*np.sin(ja2)*np.sin(ja4))
                            +3.5*np.sin(ja1)*np.cos(ja2)*np.cos(ja3),

                            -3*(np.sin(ja1)*np.sin(ja2)*np.sin(ja3)*np.cos(ja4)
                            -np.cos(ja1)*np.cos(ja4)*np.cos(ja3))
                            -3.5*(np.sin(ja1)*np.sin(ja2)*np.sin(ja3)
                            -np.cos(ja1)*np.cos(ja3)),

			     +3*(np.sin(ja1)*np.cos(ja2)*np.cos(ja4)
                            -np.sin(ja1)*np.sin(ja2)*np.cos(ja3)*np.sin(ja4)
                            -np.cos(ja1)*np.sin(ja4)*np.sin(ja3)
                            )
                            ],
                          [

                            
                            3.5*(np.cos(ja1)*np.sin(ja3)
                            +np.sin(ja1)*np.sin(ja2)*np.cos(ja3))
                            +3*(np.cos(ja1)*np.cos(ja4)*np.sin(ja3)
                            +np.sin(ja1)*np.sin(ja2)*np.cos(ja3)*np.cos(ja4)
                            +np.sin(ja1)*np.cos(ja2)*np.sin(ja4)),

                            3*(np.cos(ja1)*np.sin(ja2)*np.sin(ja4)
                            -np.cos(ja1)*np.cos(ja2)*np.cos(ja3)*np.cos(ja4))
                            -3.5*np.cos(ja1)*np.cos(ja2)*np.cos(ja3)
                            ,

                            +3*(np.cos(ja1)*np.sin(ja2)*np.sin(ja3)*np.cos(ja4)
                            +np.sin(ja1)*np.cos(ja4)*np.cos(ja3))
                            +3.5*(np.cos(ja1)*np.sin(ja2)*np.sin(ja3)
                            +np.sin(ja1)*np.cos(ja3)),

                            +3*(np.cos(ja1)*np.sin(ja2)*np.cos(ja3)*np.sin(ja4)
                            -np.sin(ja1)*np.sin(ja4)*np.sin(ja3)
                            -np.cos(ja1)*np.cos(ja2)*np.cos(ja4))
                          ],
                          [ 0,

			     -3.5*np.cos(ja3)*np.sin(ja2)
                            -3*(np.cos(ja3)*np.cos(ja4)*np.sin(ja2)
                            -np.sin(ja4)*np.cos(ja2)),

                            -3*np.sin(ja3)*np.cos(ja4)*np.cos(ja2)
                            -3.5*np.sin(ja3)*np.cos(ja2),

                            -3*np.cos(ja3)*np.sin(ja4)*np.cos(ja2)
                            -3*np.cos(ja4)*np.sin(ja2)]

        ])
        return jacobian

    def detect_target(self,image):
      mask = cv2.inRange(image, (5, 50, 50), (25, 255, 255))
      ret, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
      cnt, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      if(len(cnt)>=2):
        c = max(cnt,key=len)
        (cy, cz), radius = cv2.minEnclosingCircle(c)
        return np.array([int(cy), int(cz)])
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
          (cy, cz), radius = cv2.minEnclosingCircle(c)
      return np.array([int(cy), int(cz)])

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
