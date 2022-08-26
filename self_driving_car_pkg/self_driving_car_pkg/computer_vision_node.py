import cv2
from geometry_msgs.msg import Twist
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy 

from .Drive_Bot import Car, Debugging

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

    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)
        
    def process_data(self, data): 
        """Processes the data stream from the sensor (camera) and passes on to the 
           Self Drive Algorithm which computes and executes the appropriate control
           (Steering and speed) commands.

        Args:
            data (img_msg): image data from the camera received as a ros message
        """
        self.Debug.setDebugParameters()

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        Angle,Speed,img = self.Car.driveCar(frame)

        self.velocity.angular.z = Angle
        self.velocity.linear.x = Speed      

        cv2.imshow("Frame",img)
        cv2.waitKey(1)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()