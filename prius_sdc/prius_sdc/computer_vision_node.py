import cv2
import numpy as np
import os

from .config import config
from .Detection.Lanes.Lane_Detection import Detect_Lane
from .Detection.Signs.SignDetectionApi import detect_Signs
from .Control.special import Drive_Car

from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy 



class Video_feed_in(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image,'/camera1/image_raw',self.process_data,10)
        self.bridge = CvBridge() # converting ros images to opencv data
        self.self.prev_Mode = "Detection"
        self.self.Left_turn_iterations = 0
        self.self.Frozen_Curvature=0
        self.self.Frozen_Curvature=0

        
    def process_data(self, data): 
        frame = self.bridge.imgmsg_to_cv2(data) # performing conversion
        img = frame[0:640,402:1267] 
        img = cv2.resize(img,(320,240))
        img_orig = img.copy()
        distance, Curvature = Detect_Lane(img)
        Mode , Tracked_class = detect_Signs(img_orig,img)
        if ( (self.prev_Mode =="Detection") and (Mode=="Tracking") and (Tracked_class=="left_turn") ):
            self.prev_Mode = "Tracking"
            config.Activat_LeftTurn = True
            self.Frozen_Curvature = distance
            self.Frozen_Curvature = Curvature
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Tracking <<<<<<<<<<<<<<<<<<<<<<<<<<<")
        elif ( (self.prev_Mode =="Tracking") and (Mode=="Detection") and (Tracked_class=="left_turn") ):
            print("Left Activated")
            print("config.Activat_LeftTurn ",config.Activat_LeftTurn)
            if ((self.Left_turn_iterations % 2 ) ==0):
                self.Frozen_Curvature = self.Frozen_Curvature -1 # Move left by 1 degree 
            if(self.Left_turn_iterations==100):
                print("Left DeActivated")
                self.prev_Mode = "Detection"
                config.Activat_LeftTurn = False
                self.Left_turn_iterations = 0
            self.Left_turn_iterations = self.Left_turn_iterations + 1
            if (config.Activat_LeftTurn == True):
                distance = self.Frozen_Curvature
                Curvature = self.Frozen_Curvature
        Current_State = [distance, Curvature , img , Mode , Tracked_class]
        a,b=Drive_Car(Current_State)
        print("Angle ",a," Speed" , b)
        cv2.imshow("Frame",img)
        cv2.waitKey(1)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()