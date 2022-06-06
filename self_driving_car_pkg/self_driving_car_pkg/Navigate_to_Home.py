
import rclpy 
from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 

import cv2
from .Drive_Bot import Car, Debugging
from .config import config

from .Navigation import Navigator

class Video_feed_in(Node):
    def __init__(self):

        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.5;self.timer = self.create_timer(timer_period, self.send_cmd_vel)

        self.velocity = Twist()
        self.bridge   = CvBridge() # converting ros images to opencv data
        self.Debug    = Debugging()
        self.Car      = Car()
        
        # GPS-Navigation
        self.navigator = Navigator()

        self.satview_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.process_sat_data,10)
        self.sat_view = []

    # GPS-Navigation
    def process_sat_data(self, data): 
        self.sat_view = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
   
    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)
        
    def process_data(self, data): 

        if self.sat_view!=[]:
            self.navigator.navigate_to_home(self.sat_view)

        self.Debug.setDebugParameters()

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        Angle,Speed,img = self.Car.driveCar(frame)
        
        if config.engines_on:
            self.velocity.angular.z = Angle
            self.velocity.linear.x = Speed
        else:
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0

        cv2.imshow("Frame",img)
        cv2.waitKey(1)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()