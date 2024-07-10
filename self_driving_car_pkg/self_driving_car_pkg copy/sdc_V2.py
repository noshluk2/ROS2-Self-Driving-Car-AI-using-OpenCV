import cv2
from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy 

from .Drive_Bot import Car, Debugging

# Sat-Nav Imports
from .GPS_Navigation.Navigation import Navigator
from .config import config
# [NEW] Importing Odometry to extract robot current pose and velocity in gazebo
from nav_msgs.msg import Odometry

# [NEW] Animation Imports
from itertools import count
from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt

# [NEW] Threading Imports
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

        # creating object of navigator class
        self.navigator = Navigator()

        # Subscribing to satellite imagery provided by upper_camera/image_raw
        self.satview_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.process_sat_data,10)
        # Container to store frame retrived from overhead_view
        self.sat_view = [] 

        # [NEW]: Subscrbing to receive the robot pose in simulation
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.navigator.bot_motionplanner.get_pose,10)

        self.prius_dashcam = []

        # [NEW] Steering Animation Variables
        self.x_vals = []
        self.y_1 = []
        self.y_2 = []
        self.index = count()


    # [NEW] Steering Animation Functions
    def animate_prius_turning(self,num):
        self.x_vals.append(next(self.index))
        self.y_1.append(config.angle_orig)
        self.y_2.append(config.angle)

        plt.cla()
        plt.plot(self.x_vals,self.y_1,label = "angle")
        plt.plot(self.x_vals,self.y_2,label = "angle (rolling avg)")
        plt.legend(loc='upper left')
    # [NEW] Steering Animation Functions
    def animate(self):
        plt.style.use('fivethirtyeight')    
        self.ani = FuncAnimation(plt.gcf(), self.animate_prius_turning,interval=100)
        plt.tight_layout()
        plt.show()


    # Processing data received from Satellite to be used for Satellite navigation
    def process_sat_data(self, data):
        self.sat_view = self.bridge.imgmsg_to_cv2(data,'bgr8')


        
    def process_data(self, data): 
        """Self Drive System with autonomous navigation system Integrated.
        
        What do I mean by that?
        Well! The Prius can now go to any ROI (a.k.a house of your selection) on the map just like in the GPS

        Limitation: The sad part :( Its vision based navigation so only the world thats in view of the satellite
        will be navigatable (I am not sure, if its even a word :D)

        Args:
            data (img_msg): image data from the camera received as a ros message
        """

        self.Debug.setDebugParameters()

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        if  ( (config.enable_SatNav) and ( self.sat_view!=[] ) ):
            # Adding prius_dashcam to be passed as an argument to navigat_to_home
            #       So that we can see sdc_First Person View of navigating to home
            if self.prius_dashcam==[]:
                self.prius_dashcam = frame
            self.navigator.navigate_to_home(self.sat_view,self.prius_dashcam)

        # [NEW]: Self Drive in Action being displayed in prius_dashcam member variable
        Angle,Speed,self.prius_dashcam = self.Car.driveCar(frame)

        # [NEW]: No Road Speed Limit or No Intersection... Speed is dictated by Sat Nav
        if config.enable_SatNav:
            if ((self.Car.Tracked_class=="Unknown") and (self.Car.Traffic_State=="Unknown")):
                Speed = float(self.navigator.bot_motionplanner.vel_linear_x)

        # if engines_on normal mode,
        #   else stop car
        if config.engines_on:
            # [NEW]: Sat-Nav only influences steering in case of sharp turns
            if (config.enable_SatNav and self.navigator.bot_motionplanner.car_turning):
                self.velocity.angular.z = self.navigator.bot_motionplanner.vel_angular_z
            else:
                self.velocity.angular.z = Angle
            self.velocity.linear.x = Speed
        else:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0 
        
        # publishing updated velocity object
        self.publisher.publish(self.velocity)

        # [NEW]: Enable Sat-NaV switch 
        if not config.enable_SatNav:
            cv2.imshow("Frame",self.prius_dashcam)
            cv2.waitKey(1)
        else:
            try:
                cv2.destroyWindow("Frame")
            except:
                pass
        if config.debugging:
            print('number of current threads is ', threading.active_count())
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  # [NEW]: Animate Steering to see how rolling average smoothes the lane assist
  if config.animate_steering:
    concurrent.futures.ThreadPoolExecutor().submit(image_subscriber.animate)
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()