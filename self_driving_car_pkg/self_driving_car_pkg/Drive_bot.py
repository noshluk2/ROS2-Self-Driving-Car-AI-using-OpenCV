import cv2
from .Detection.Lanes.lane_detection import detect_lanes
from numpy import interp
class Control():
    def __init__(self):
        self.angle = 0.0
        self.speed = 80
    
    def follow_lane(self,max_sane_dist,dist,curv):
        self.speed = 80

        max_turn_angle = 90; max_turn_angle_neg =-90; req_turn_angle = 0

        if ((dist>max_sane_dist)or (dist < (-1*max_sane_dist))):
            if(dist>max_sane_dist):
                req_turn_angle = max_turn_angle + curv
            else:
                req_turn_angle = max_turn_angle_neg + curv
        else:
            car_offset = interp(dist,[-max_sane_dist,max_sane_dist],[-max_turn_angle,max_turn_angle])
            req_turn_angle = car_offset + curv
        
        #handle overflow
        if ((req_turn_angle>max_turn_angle)or (req_turn_angle<max_turn_angle_neg)):
            if (req_turn_angle>max_turn_angle):
                req_turn_angle = max_turn_angle
            else:
                req_turn_angle = max_turn_angle_neg
        # Handle max car turn ability
        self.angle = interp(req_turn_angle,[max_turn_angle_neg,max_turn_angle],[-45,45])

    def drive(self,Current_State):
        [dist,curv,img] = Current_State

        if ((dist!=1000)and (curv!= 1000)):
            self.follow_lane(img.shape[1]/4,dist,curv)
        else:
            self.speed = 0.0 # Stop the car
        # Interpolating the angle and speed from real world to motor worlld
        self.angle = interp(self.angle,[-45,45],[0.5,-0.5])
        self.speed = interp(self.speed,[30,90] ,[1,2])

        


class Car():
    def __init__(self):
        self.Control = Control()

    def display_state(self,frame_disp,angle_of_car,current_speed):

        # Translate [ ROS Car Control Range ===> Real World angle and speed  ]
        angle_of_car  = interp(angle_of_car,[-0.5,0.5],[45,-45])
        if (current_speed !=0.0):
            current_speed = interp(current_speed  ,[1  ,   2],[30 ,90])

        ###################################################  Displaying CONTROL STATE ####################################

        if (angle_of_car <-10):
            direction_string="[ Left ]"
            color_direction=(120,0,255)
        elif (angle_of_car >10):
            direction_string="[ Right ]"
            color_direction=(120,0,255)
        else:
            direction_string="[ Straight ]"
            color_direction=(0,255,0)

        if(current_speed>0):
            direction_string = "Moving --> "+ direction_string
        else:
            color_direction=(0,0,255)


        cv2.putText(frame_disp,str(direction_string),(20,40),cv2.FONT_HERSHEY_DUPLEX,0.4,color_direction,1)

        angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "mph ]"
        cv2.putText(frame_disp,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)
    

    def drive_car(self,frame):

        img = frame[0:640,238:1042]
        # resizing to minimize computation time while still achieving comparable results
        img = cv2.resize(img,(320,240))

        distance, Curvature = detect_lanes(img)
        Current_State = [distance,Curvature,img]
        
        self.Control.drive(Current_State)

        self.display_state(img,self.Control.angle,self.Control.speed)

        return self.Control.angle,self.Control.speed,img