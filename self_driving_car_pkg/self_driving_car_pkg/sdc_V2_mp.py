
import rclpy 
from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 

import cv2
from .Drive_Bot import Car, Debugging
from .config import config

from .GPS_Navigation.Navigation import Navigator

from nav_msgs.msg import Odometry


from itertools import count
from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
import concurrent.futures
import threading
from multiprocessing import Process,Array
import multiprocessing
import time
import ctypes

#sat_view_global = []
class Video_feed_in(Node):

    def __init__(self):

        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)

        self.velocity = Twist()
        self.bridge   = CvBridge() # converting ros images to opencv data
        self.Debug    = Debugging()
        self.Car      = Car()
        
        # GPS-Navigation
        self.navigator = Navigator()

        self.satview_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.process_sat_data,10)
        self.sat_view = []
        
        self.prius_dashcam = []

        # Testing 
        self.x_vals = []
        self.y_1 = []
        self.y_2 = []

        self.index = count()
        #self.chachu = 0


        # Subscrbing to receive the robot pose in simulation
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.navigator.bot_motionplanner.get_pose,10)


    def animate_prius_turning(self,num):

        self.x_vals.append(next(self.index))
        self.y_1.append(config.angle_orig)
        self.y_2.append(config.angle)

        plt.cla()
        plt.plot(self.x_vals,self.y_1,label = "angle")
        plt.plot(self.x_vals,self.y_2,label = "angle (rolling avg)")
        plt.legend(loc='upper left')
    
    def animate(self):
        plt.style.use('fivethirtyeight')    
        self.ani = FuncAnimation(plt.gcf(), self.animate_prius_turning,interval=100)
        plt.tight_layout()
        
        plt.show()
        
    # GPS-Navigation
    def process_sat_data(self, data):
        #global sat_view_global        
        self.sat_view = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
        #print("type(self.sat_view) = ",type(self.sat_view))
        #print("self.sat_view = ",self.sat_view.dtype)
        #sat_view_global = self.sat_view
        #self.chachu = Array(ctypes.c_double,self.sat_view)


    def Sat_Nav(self):

        while(1):
            #time.sleep(1)
            print("################################################################### ")
            print("############################ No sat viw")
            print("")
            
            #if self.sat_view!=[]:
            #if self.chachu!=0:
            if sat_view_global!=[]:
                print("party")
                self.navigator.navigate_to_home(sat_view_global)

    def initialize_process_SatNav(self):
        process = Process(target= self.Sat_Nav)
        process.start()


    def process_data(self, data): 

        self.Debug.setDebugParameters()

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        #p = 0
        if ( (config.enable_SatNav) and (self.sat_view!=[]) ):
            #with concurrent.futures.ProcessPoolExecutor(max_workers=2) as  executor:
            #    executor.submit(self.navigator.navigate_to_home(self.sat_view))
            
            #p = Process(target= self.navigator.navigate_to_home, args=(self.sat_view,))
            if self.prius_dashcam==[]:
                self.prius_dashcam = frame
                
            self.navigator.navigate_to_home(self.sat_view,self.prius_dashcam)
            #p.start()
            #p.join()

        Angle,Speed,self.prius_dashcam = self.Car.driveCar(frame)
        print("Original Speed = ",Speed)

        # No Road Speed Limit or No Intersection... Speed is dictated by Sat Nav
        if ((self.Car.Tracked_class=="Unknown") and (self.Car.Traffic_State=="Unknown")):
            Speed = float(self.navigator.bot_motionplanner.vel_linear_x)
        print("Current Speed = ",Speed)
        print("self.Car.Traffic_State= ",self.Car.Traffic_State)
        
        if config.engines_on:
            if self.navigator.bot_motionplanner.car_turning:
                self.velocity.angular.z = self.navigator.bot_motionplanner.vel_angular_z
            else:
                self.velocity.angular.z = Angle

            self.velocity.linear.x = Speed
        else:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0

        self.publisher.publish(self.velocity)

        if not config.enable_SatNav:
            cv2.imshow("Frame",self.prius_dashcam)
            cv2.waitKey(1)
        
        #if p!=0:
        #    p.join()
        print('number of current threads is ', threading.active_count()) 
        
        #print('number of current processes is ', multiprocessing.active_children()) 
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  
  #concurrent.futures.ProcessPoolExecutor(max_workers=1).submit(image_subscriber.looperthepooper)
  #image_subscriber.initialize_process_SatNav()

  if config.animate_steering:
    concurrent.futures.ThreadPoolExecutor().submit(image_subscriber.animate)

  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
    main()