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

    # Processing data received from Satellite to be used for Satellite navigation
    def process_sat_data(self, data):
        self.sat_view = self.bridge.imgmsg_to_cv2(data,'bgr8')


        
    def process_data(self, data): 

        self.Debug.setDebugParameters()

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        # Enable access to Sat-Nav feature (once data from overhead_view arrives)
        if ( self.sat_view!=[] ):
            self.navigator.navigate_to_home(self.sat_view)

        Angle,Speed,img = self.Car.driveCar(frame)

        # if engines_on normal mode,
        #   else stop car
        if config.engines_on:
            self.velocity.angular.z = Angle
            self.velocity.linear.x = Speed 
        else:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0 
        
        # publishing updated velocity object
        self.publisher.publish(self.velocity)

        cv2.imshow("Frame",img)
        cv2.waitKey(1)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()