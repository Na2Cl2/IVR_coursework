
import numpy as np

def fk(joints):
    a,b,c,d = joints
    x = np.sin(a)*(np.sin(b)*np.cos(c)*(3*np.cos(d)+3.5)+3*np.cos(b)*np.sin(d)) + np.cos(a)*np.sin(c)*(3*np.cos(d)+3.5)
    y = np.cos(a)*(np.sin(b)*np.cos(c)*(-3*np.cos(d)-3.5) - 3*np.cos(b)*np.sin(d)) + np.sin(a)*np.sin(c)*(3*np.cos(d)+3.5)
    z = np.cos(b)*np.cos(c)*(3*np.cos(d)+3.5) - 3*np.sin(b)*np.sin(d) + 2.5
    end_effector = np.array([x,y,z])
    return end_effector

def calculated_forward_kinematics(joints):
    ja1 = joints[0]
    ja2 = joints[1]
    ja3 = joints[2]
    ja4 = joints[3]
    #ja1, ja2, ja3, ja4 = detect_angles_blob(self,image1,image2)
    end_effector = np.array([3 * np.sin(ja1) * np.sin(ja2) * np.cos(ja3) * np.cos(ja4)
                                  + 3.5 * np.sin(ja1) * np.sin(ja2) * np.cos(ja3)
                                  + 3 * np.cos(ja1) * np.cos(ja4) *np.sin(ja3)
                                  + 3.5 * np.cos(ja1) * np.sin(ja3)
                                  + 3 * np.sin(ja1) * np.cos(ja2)*np.sin(ja4),

                                  -3 * np.cos(ja1) * np.sin(ja2) * np.cos(ja3) * np.cos(ja4)
                                  - 3.5 * np.cos(ja1) * np.sin(ja2) * np.cos(ja3)
                                  + 3 * np.sin(ja1) * np.cos(ja4) * np.sin(ja3)
                                  + 3.5 * np.sin(ja1) * np.sin(ja3)
                                  - 3 * np.cos(ja1) * np.cos(ja2)*np.sin(ja4),

                                  3 * np.cos(ja2) * np.cos(ja3) * np.cos(ja4)
                                  + 3.5 * np.cos(ja2) * np.cos(ja3) - 3 * np.sin(ja2) * np.sin(ja4) + 2.5
                                  ])
    return end_effector

if __name__ == "__main__":
    test = [[0.6,1,0.6,-0.7] 	    #1
           ,[1,-0.6,0.3,0.5] 	    #2
           ,[1.5,1,-1,-1]	    #3
           ,[-1.6,0.6,0.4,-1]	    #4
           ,[-1.6,1.2,-0.9,-0.8]   #5
           ,[1.8,0.6,1.0,-0.6]     #6
           ,[-0.8,-1.4,-0.1,0.6]   #7
           ,[1,1,1,-1]		    #8
           ,[1.2,0.4,1.8,0.5]	    #9
           ,[-1,1,-0.5,0.5]]	    #10

    test2 = [[0.1,0.2,0.2,0.2],[0.2,0.3,0.3,0.3],[0.3,0.4,0.4,0.4],[0.4,0.5,0.5,-0.5],[0.5,0.6,-0.6,0.6],[0.6,0.7,-0.7,0.7],[0.7,-0.8,0.8,0.8],[0.8,0.9,0.9,-0.9],[0.9,-1,1,1],[1,1.1,1.1,1.1]]
    #print("Joints: ",j," Position: ",np.round(fk(j),2),np.round(calculated_forward_kinematics(j),2))
    for j in test2:
        print("Joints: ",j," Position: ",np.round(fk(j),2),np.round(calculated_forward_kinematics(j),2))
