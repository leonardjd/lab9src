#!/usr/bin/python3

import rospy
import time
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

def distance(px1,px2):
    m = 2091.17
    b = -0.53
    return b + m/(px2 - px1)

class TakePhoto:
    def __init__(self):
        self.image_received = False

        img_topic = "/raspicam_node/image/compressed"
        img_topic = "/camera/image/compressed"
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.callback, queue_size = 10)

    def callback(self, data):
        self.image_received = True
        np_arr = np.frombuffer(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
    def get_img(self):
        if self.image_received == False:
           print("None")
           return
        return self.img
    def save_img(self, img_title):
        if self.image_received == False:
            print("None")
            return
        cv2.imwrite(img_title, self.img)
    def disp_img(self, img_title):
    	if self.image_received == False:
    	     print("None")
    	     return
    	cv2.imshow(img_title, self.img)

class CmdListener:
    def __init__(self):
        self.cmd = ""
        self.cmd_sub = rospy.Subscriber("/master_cmd", String, self.callback)
    def callback(self, cmd):
        self.cmd = cmd.data
    def get_msg(self):
        return self.cmd
    def setup_order(self):
        while self.get_msg() == "HIGH":
                pass
        print("High ")
        while self.get_msg() == "LOW":
                pass
        print("Low ")

if __name__ == "__main__":
    i = 0
    rospy.init_node("LED_Tracking")
    feedback_pub = rospy.Publisher("/feedback", String, queue_size = 1)
    camera = TakePhoto()
    cmd_listener = CmdListener()
    rate = rospy.Rate(10)
    NX = 40                             # number of pixels to process, 30 works
    led_on = None
    led_off = None
    time.sleep(2)
    feedback_pub.publish(str(0))
    #cmd_listener.setup_order()
    print("Beginning cmd_listener ",cmd_listener.get_msg())
    while not rospy.is_shutdown():
        cmd_listener.setup_order()
        if cmd_listener.get_msg() == "HIGH":
            time.sleep(0.45)
            led_on = camera.get_img()
            print("***HIGH***")
            while cmd_listener.get_msg() == "HIGH":
                pass
        if cmd_listener.get_msg() == "LOW":
            time.sleep(0.45)
            led_off = camera.get_img()
            print("***LOW***")
            while cmd_listener.get_msg() == "LOW":
                pass

        if (led_on is None or led_off is None):
            diff = np.zeros([240,320], dtype=np.int32)
            if(led_on == None):
                print("led_on ", led_on)
            if(led_off == None):
                print("led_off ", led_off)
            break
        else:
            diff = cv2.absdiff(led_on, led_off)
            diffB = 2*cv2.blur(diff, (3,3))    #Blured image
            mx_diffB = np.amax(diffB)
            cv2.imshow("Diff", diffB)
            idx = np.argmax(diffB, axis=0)		#x column max vector
            val = np.amax(diffB, axis=0)		#max value of image
            ival = np.argsort(val)[::-1]		#sort max column values, decending
            diffV = np.reshape(diffB, [320*240])
            AI = np.flip(np.argsort(diffB,axis=None))[0:NX] 
            AZ = np.zeros([NX,5], dtype=np.int32)     #pixel Value, linear Coordinate, X Coordinate, Y Coordinate, Cluster
            AZ[:,0] = diffV[AI]
            AZ[:,1] = AI
            AZ[:,2] = AI/320
            AZ[:,3] = AI%320
            AZ2i = np.argsort(AZ[:,3])
            AZZ = AZ[AZ2i,:]
            clus = 1
            pt0 = 0;
            for i in range(NX):
                 if( (AZZ[i,3] - AZZ[pt0,3]) < 2): #look for a  break in the x coordinates
                     AZZ[i,4] = clus 
                 else:                             #found a break in the x values (except final cluster)
                     clus += 1
                     AZZ[i,4] = clus
                 pt0 = i
            
            print(AZZ)
            
            led_on = 0
            led_off = 0
            
            i+=1
            # Calculate the X and Y average pixel values of each cluster
            #Clust values                cluster, X, Y, xmin, xmax, ymin, ymax
            Clust = np.zeros((clus,7), dtype=np.double)
            for i in range(clus):
               Clust[i,0] = i+1
               clus_ind = np.where(AZZ[:,4] == [i+1] )[0]
               print(i+1)
               print(clus_ind)
               Clust[i,1] = np.dot( AZZ[clus_ind,3],AZZ[clus_ind,0])/np.sum(AZZ[clus_ind,0])
               Clust[i,2] = np.dot( AZZ[clus_ind,2],AZZ[clus_ind,0])/np.sum(AZZ[clus_ind,0])
               Clust[i,3] = np.amin(AZZ[clus_ind,3])
               Clust[i,4] = np.amax(AZZ[clus_ind,3])
               Clust[i,5] = np.amin(AZZ[clus_ind,2])
               Clust[i,6] = np.amax(AZZ[clus_ind,2])
               print("X",Clust[i,1], " Xmin",Clust[i,3], "Xmax", Clust[i,4])
               print("Y",Clust[i,2], " Ymin",Clust[i,5], "Ymax", Clust[i,6])
               print()
            
            print("---------------------------------------------------------")
            print("Left Front LED Coordinate ", Clust[0,1],", ",Clust[0,2])
            print("Right Front LED Coordinate", Clust[2,1],", ",Clust[2,2])
            print("Rear LED Coordinate       ", Clust[1,1],", ",Clust[1,2])
            print("Distance ", distance(Clust[0,1],Clust[2,1]))
            print("---------------------------------------------------------")
            
            if(clus == 3):
                cv2.waitKey(0)
                break
            cv2.waitKey(0)
        rate.sleep()
    print("finished running")
    feedback_pub.publish(str(1))
        

