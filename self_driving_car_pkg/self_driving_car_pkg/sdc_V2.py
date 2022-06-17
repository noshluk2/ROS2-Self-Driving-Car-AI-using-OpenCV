#Ros Imports
import rclpy 
from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 

# Self-Drive Imports
import cv2
from .Drive_Bot import Car, Debugging
from .config import config

# Sat-Nav System Imports
from .GPS_Navigation.Navigation import Navigator
from nav_msgs.msg import Odometry

# Animation Imports
from itertools import count
from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt

# Threading Imports
import concurrent.futures
import threading

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

        # Subscrbing to receive the robot pose in simulation
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.navigator.bot_motionplanner.get_pose,10)

        # Steering Animation Variables
        self.x_vals = []
        self.y_1 = []
        self.y_2 = []
        self.index = count()

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
        
    # Processing data received from Satellite to be used for Satellite navigation
    def process_sat_data(self, data):
        self.sat_view = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

    def process_data(self, data): 

        self.Debug.setDebugParameters()

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        if ( (config.enable_SatNav) and (self.sat_view!=[]) ):
            if self.prius_dashcam==[]:
                self.prius_dashcam = frame
            self.navigator.navigate_to_home(self.sat_view,self.prius_dashcam)

        Angle,Speed,self.prius_dashcam = self.Car.driveCar(frame)
        
        print("Original Speed = ",Speed)
        # No Road Speed Limit or No Intersection... Speed is dictated by Sat Nav
        if config.enable_SatNav:
            if ((self.Car.Tracked_class=="Unknown") and (self.Car.Traffic_State=="Unknown")):
                Speed = float(self.navigator.bot_motionplanner.vel_linear_x)
        print("Current Speed = ",Speed)
        
        if config.engines_on:
            if (config.enable_SatNav and self.navigator.bot_motionplanner.car_turning):
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
        else:
            try:
                cv2.destroyWindow("Frame")
            except:
                pass
        print('number of current threads is ', threading.active_count()) 
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  if config.animate_steering:
    concurrent.futures.ThreadPoolExecutor().submit(image_subscriber.animate)
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
    main()