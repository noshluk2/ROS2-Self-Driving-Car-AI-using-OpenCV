import cv2
from numpy import interp
import numpy as np

from .config import config
from .Detection.Lanes.Lane_Detection import detect_Lane
from .Detection.Signs.SignDetectionApi import detect_Signs
# from .Detection.Signs.a_Localization.TLD import detect_TrafficLight
from .Detection.Signs.a_Localization.ObjDet_cascade_classified import detect_TrafficLights
from .Control.special import Drive_Car
from .Control.Control_TrafficL_Nd_LeftTurn import Control

from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy 



def nothing(x):
    pass

cv2.namedWindow('CONFIG')
# create switch for ON/OFF functionality
debugging_SW = 'Debug'
cv2.createTrackbar(debugging_SW, 'CONFIG',False,True,nothing)
# create switch for ON/OFF functionality
debuggingLane_SW = 'Debug Lane'
cv2.createTrackbar(debuggingLane_SW, 'CONFIG',False,True,nothing)
# create switch for ON/OFF functionality
debuggingSigns_SW = 'Debug Sign'
cv2.createTrackbar(debuggingSigns_SW, 'CONFIG',False,True,nothing)
# create switch for ON/OFF functionality
debuggingTL_SW = 'Debug TL'
cv2.createTrackbar(debuggingTL_SW, 'CONFIG',False,True,nothing)

control = Control() 
class Video_feed_in(Node):
    def __init__(self):

        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.5;self.timer = self.create_timer(timer_period, self.send_cmd_vel)

        self.velocity=Twist()
        self.bridge = CvBridge() # converting ros images to opencv data
        


    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)
        
    def process_data(self, data): 

        # #############################  DEBUG CONTROLS #######################################
        # get current positions of four trackbars
        debug = cv2.getTrackbarPos(debugging_SW,'CONFIG')
        debugLane = cv2.getTrackbarPos(debuggingLane_SW,'CONFIG')
        debugSign = cv2.getTrackbarPos(debuggingSigns_SW,'CONFIG')
        debugTrafficLights = cv2.getTrackbarPos(debuggingTL_SW,'CONFIG')

        if debug:
            config.debugging = True
        else:
            config.debugging = False            
        if debugLane:
            config.debugging_Lane = True
        else:
            config.debugging_Lane = False    
        if debugSign:
            config.debugging_Signs = True
        else:
            config.debugging_Signs = False           
        if debugTrafficLights:
            config.debugging_TrafficLights = True
        else:
            config.debugging_TrafficLights = False          
        print("config.debugging = ",config.debugging)
        print("config.debugging_lane = ",config.debugging_Lane)
        print("config.debugging_Sign = ",config.debugging_Signs)
        print("config.debugging_TrafficLights = ",config.debugging_TrafficLights)
        # #############################  DEBUG CONTROLS #######################################

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
        img = frame[0:640,238:1042]
        img_b4_resiz = img.copy()
        img = cv2.resize(img,(320,240))
        img_orig = img.copy()

        #Traffic_State = detect_TrafficLight(img,frame_draw)
        Traffic_State, CloseProximity = detect_TrafficLights(img_orig.copy())

        distance, Curvature = detect_Lane(img)
        Mode , Tracked_class = detect_Signs(img_orig,img)

        distance,Curvature, Detected_LeftTurn, Activat_LeftTurn = control.Obey_LeftTurn(distance,Curvature,Mode,Tracked_class)

        Current_State = [distance, Curvature , img , Mode , Tracked_class]
        a,b = Drive_Car(Current_State)

        a,b = control.OBEY_TrafficLights(a,b,Traffic_State,CloseProximity)

        angle_of_car = interp(a,[30,120],[-45,45])
        current_speed = b

        angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "mph ]"
        cv2.putText(img,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)

        cv2.putText(img,"Traffic Light State = [ "+Traffic_State+" ] ",(20,60),cv2.FONT_HERSHEY_COMPLEX,0.35,255)
        
        if (Tracked_class=="left_turn"):
            font_Scale = 0.30
            if (Detected_LeftTurn):
                Tracked_class = Tracked_class + " : Detected { True } "
            else:
                Tracked_class = Tracked_class + " : Activated { "+ str(Activat_LeftTurn) + " } "
        else:
            font_Scale = 0.35
        cv2.putText(img,"Sign Detected ==> "+str(Tracked_class),(20,85),cv2.FONT_HERSHEY_COMPLEX,font_Scale,(255,0,0),1)

        a=interp(a,[30,120],[0.5,-0.5])
        if (b!=0):
            b=interp(b,[30,90],[1,2])
        self.velocity.linear.x = float(b)        
        self.velocity.angular.z = a

        cv2.imshow("Frame",img)
        cv2.waitKey(1)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()