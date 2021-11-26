import cv2
from .Detection.Lanes.lane_detection import detect_lanes
from .Detection.Signs.sign_detection import detect_signs
from .Detection.TrafficLights.TL_Detection import detect_TrafficLights

from numpy import interp
class Control():
    def __init__(self):
        # Lane assist Variable
        self.angle = 0.0
        self.speed = 80
        # Cruise_Control Variable
        self.prev_Mode = "Detection"
        self.IncreaseTireSpeedInTurns = False
        # Nav T-Junc Variable
        self.prev_Mode_LT = "Detection"
        self.Left_turn_iterations = 0
        self.Frozen_Angle = 0
        self.Detected_LeftTurn = False
        self.Activat_LeftTurn = False
        # Cross. Intersection Variables
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False
        self.crossing_intersection_timer = 0
    
    def follow_lane(self,max_sane_dist,dist,curv,mode,tracked_class):
        #2. Cruise control speed adjusted to match road speed limit
        if((tracked_class!=0) and (self.prev_Mode == "Tracking") and (mode == "Detection")):
            if  (tracked_class =="speed_sign_30"):
                self.speed = 30
            elif(tracked_class =="speed_sign_60"):
                self.speed = 60
            elif(tracked_class =="speed_sign_90"):
                self.speed = 90
            elif(tracked_class =="stop"):
                self.speed = 0
                print("Stopping Car !!!")
            
        self.prev_Mode = mode # Set prevMode to current Mode

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
        if (self.IncreaseTireSpeedInTurns and (tracked_class !="left_turn")):
            if(self.angle>30):
                car_speed_turn = interp(self.angle,[30,45],[80,100])
                self.speed = car_speed_turn
            elif(self.angle<(-30)):
                car_speed_turn = interp(self.angle,[-45,-30],[100,80])
                self.speed = car_speed_turn


    def Obey_LeftTurn(self,mode):

        self.speed = 50
        # Car starts tracking left turn...
        if ( (self.prev_Mode_LT =="Detection") and (mode=="Tracking")):
            self.prev_Mode_LT = "Tracking"
            self.Detected_LeftTurn = True 
        elif ( (self.prev_Mode_LT =="Tracking") and (mode=="Detection")):
            self.Detected_LeftTurn = False
            self.Activat_LeftTurn = True
            # Move left by 7 degree every 20th iteration after a few waiting a bit 
            if ( ((self.Left_turn_iterations % 20 ) ==0) and (self.Left_turn_iterations>100) ):
                self.Frozen_Angle = self.Frozen_Angle -7
            
            # After a time period has passed [ De-Activate Left Turn + Reset Left Turn Variables ]
            if(self.Left_turn_iterations==250):
                self.prev_Mode_LT = "Detection"
                self.Activat_LeftTurn = False
                self.Left_turn_iterations = 0
                
            self.Left_turn_iterations = self.Left_turn_iterations + 1

        # Angle of car adjusted here
        if (self.Activat_LeftTurn or self.Detected_LeftTurn):
            #Follow previously Saved Route
            self.angle = self.Frozen_Angle

    def OBEY_TrafficLights(self,Traffic_State,CloseProximity):
        # A: Car was close to TL which was signalling stop
        if((Traffic_State == "Stop") and CloseProximity):
            self.speed = 0 # Stopping car
            self.STOP_MODE_ACTIVATED = True 
        else:
            # B: Car is Nav. Traffic Light
            if (self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED):
                # B-1: Car was stopped at Red and now TL has turned green
                if (self.STOP_MODE_ACTIVATED and (Traffic_State=="Go")):
                    self.STOP_MODE_ACTIVATED = False
                    self.GO_MODE_ACTIVATED = True
                # B-2: Stop Mode is activated so car cannot move
                elif(self.STOP_MODE_ACTIVATED):
                    self.speed = 0
                # B-3: Go Mode is activated --> Car moves straight ignoring lane assist
                #                               for a few moments while it crosses intersection
                elif(self.GO_MODE_ACTIVATED):
                    self.angle = 0.0
                    self.speed = 80 # Set default speed
                                        
                    if(self.crossing_intersection_timer==200):
                        self.GO_MODE_ACTIVATED = False
                        print("Intersection Crossed !!!")
                        self.crossing_intersection_timer = 0 #Reset

                    self.crossing_intersection_timer = self.crossing_intersection_timer + 1

    def drive(self,Current_State):
        #Unpacking new with the old
        [dist,curv,img,mode,tracked_class,traffic_state,proximity_status] = Current_State

        if ((dist!=1000)and (curv!= 1000)):
            self.follow_lane(img.shape[1]/4,dist,curv,mode,tracked_class)
        else:
            self.speed = 0.0 # Stop the car
            
        if (tracked_class == "left_turn"):
            self.Obey_LeftTurn(mode)

        # Integrating the Intersection Nav. control.
        self.OBEY_TrafficLights(traffic_state,proximity_status)

            
        # Interpolating the angle and speed from real world to motor worlld
        angle_motor = interp(self.angle,[-45,45],[0.5,-0.5])
        if (self.speed!=0):
            speed_motor = interp(self.speed,[30,90] ,[1,2])
        else:
            speed_motor = 0.0

        return angle_motor,speed_motor

class Car():
    def __init__(self):
        self.Control = Control()

    def display_state(self,frame_disp,angle_of_car,current_speed,tracked_class,Traffic_State="",close_proximity=False):

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
        
        if (tracked_class=="left_turn"):
            font_Scale = 0.32
            if (self.Control.Detected_LeftTurn):
                tracked_class = tracked_class + " : Detected { True } "
            else:
                tracked_class = tracked_class + " : Activated { "+ str(self.Control.Activat_LeftTurn) + " } "
        else:
            font_Scale = 0.37
        cv2.putText(frame_disp,"Sign Detected ==> "+str(tracked_class),(20,80),cv2.FONT_HERSHEY_COMPLEX,font_Scale,(0,255,255),1)    

        if Traffic_State != "":
            cv2.putText(frame_disp,"Traffic Light State = [ "+Traffic_State+" ] ",(20,60),cv2.FONT_HERSHEY_COMPLEX,0.35,255)
            if close_proximity:
                cv2.putText(frame_disp," (P.Warning!) ",(220,75),cv2.FONT_HERSHEY_COMPLEX,0.35,(0,0,255))
                

    def drive_car(self,frame):

        img = frame[0:640,238:1042]
        # resizing to minimize computation time while still achieving comparable results
        img = cv2.resize(img,(320,240))

        img_orig = img.copy()
        
        # ================================ [ Detection ] ============================================
        distance, Curvature = detect_lanes(img)
        
        traffic_state,proximity_status = detect_TrafficLights(img_orig.copy(),img)

        mode, tracked_class = detect_signs(img_orig,img)

        # ========================= [ Updating Current State ] =======================================        
        Current_State = [distance,Curvature,img,mode,tracked_class,traffic_state,proximity_status]
        
        # ================================= [ Control ] ============================================
        angle_m,speed_m = self.Control.drive(Current_State)

        # ================================= [ Display ] ============================================
        self.display_state(img,angle_m,speed_m,tracked_class,traffic_state,proximity_status)
        

        return angle_m,speed_m,img