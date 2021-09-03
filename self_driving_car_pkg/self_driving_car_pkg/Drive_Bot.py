from .Detection.Lanes.Lane_Detection import detect_Lane
from .Detection.Signs.SignDetectionApi import detect_Signs
from .Detection.TrafficLights.TrafficLights_Detection import detect_TrafficLights
import cv2
from numpy import interp
from .config import config

class Debugging:

    def __init__(self):
        self.TL_Created = False
        self.Lan_Created = False

    def nothing(self,x):
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


    def setDebugParameters(self):
        # get current positions of four trackbars
        debug = cv2.getTrackbarPos(self.debugging_SW,'CONFIG')
        debugLane = cv2.getTrackbarPos(self.debuggingLane_SW,'CONFIG')
        debugSign = cv2.getTrackbarPos(self.debuggingSigns_SW,'CONFIG')
        debugTrafficLights = cv2.getTrackbarPos(self.debuggingTL_SW,'CONFIG')


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

        if config.debugging_TrafficLights:
            
            debuggingTLConfig_SW = 'Debug Config'
            if not self.TL_Created:
                self.TL_Created = True
                cv2.namedWindow('CONFIG_TL')
                cv2.createTrackbar(debuggingTLConfig_SW, 'CONFIG_TL',False,True,self.nothing)

            debugTL_Config = cv2.getTrackbarPos(debuggingTLConfig_SW,'CONFIG_TL')

            if debugTL_Config:
                config.debugging_TL_Config = True
            else:
                config.debugging_TL_Config = False

        else:
            self.TL_Created = False
            cv2.destroyWindow('CONFIG_TL')

        
        if config.debugging_Lane:
            
            debuggingLANEConfig_SW = 'Debug (Stage)'
            if not self.Lan_Created:
                self.Lan_Created = True
                cv2.namedWindow('CONFIG_LANE')
                cv2.createTrackbar(debuggingLANEConfig_SW, 'CONFIG_LANE',0,3,self.nothing)

            debugLane_Config = cv2.getTrackbarPos(debuggingLANEConfig_SW,'CONFIG_LANE')

            if debugLane_Config == 0:
                config.debugging_L_ColorSeg = True
                config.debugging_L_Est = config.debugging_L_Cleaning = config.debugging_L_LaneInfoExtraction = False                    
            elif debugLane_Config == 1:
                config.debugging_L_Est = True
                config.debugging_L_ColorSeg = config.debugging_L_Cleaning = config.debugging_L_LaneInfoExtraction = False   
            elif debugLane_Config == 2:
                config.debugging_L_Cleaning = True
                config.debugging_L_ColorSeg = config.debugging_L_Est = config.debugging_L_LaneInfoExtraction = False   
            elif debugLane_Config == 3:
                config.debugging_L_LaneInfoExtraction = True
                config.debugging_L_ColorSeg = config.debugging_L_Est = config.debugging_L_Cleaning = False

        else:
            self.Lan_Created = False
            cv2.destroyWindow('CONFIG_LANE')        

class Control:

    def __init__(self):
        self.prev_Mode = "Detection"


        self.prev_Mode_LT = "Detection"
        self.car_speed = 80
        self.angle_of_car = 0

        self.Left_turn_iterations = 0
        self.Frozen_Angle = 0
        self.Detected_LeftTurn = False
        self.Activat_LeftTurn = False        

        self.TrafficLight_iterations = 0
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False

    def follow_Lane(self,Max_Sane_dist,distance,curvature , Mode , Tracked_class):

        IncreaseTireSpeedInTurns = True

        if((Tracked_class!=0) and (self.prev_Mode == "Tracking") and (Mode == "Detection")):
            if  (Tracked_class =="speed_sign_30"):
                self.car_speed = 30
            elif(Tracked_class =="speed_sign_60"):
                self.car_speed = 60
            elif(Tracked_class =="speed_sign_90"):
                self.car_speed = 90
            elif(Tracked_class =="stop"):
                self.car_speed = 0
            
        self.prev_Mode = Mode # Set prevMode to current Mode
        
        Max_turn_angle_neg = -90
        Max_turn_angle = 90

        CarTurn_angle = 0

        if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ):
            # Max sane distance reached ---> Max penalize (Max turn Tires)
            if(distance > Max_Sane_dist):
                #Car offseted left --> Turn full wheels right
                CarTurn_angle = Max_turn_angle + curvature
            else:
                #Car Offseted right--> Turn full wheels left
                CarTurn_angle = Max_turn_angle_neg + curvature
        else:
            # Within allowed distance limits for car and lane
            # Interpolate distance to Angle Range
            Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90])
            CarTurn_angle = Turn_angle_interpolated + curvature

        # Handle Max Limit [if (greater then either limits) --> set to max limit]
        if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
            if(CarTurn_angle > Max_turn_angle):
                CarTurn_angle = Max_turn_angle
            else:
                CarTurn_angle = -Max_turn_angle

        #angle = CarTurn_angle
        angle = interp(CarTurn_angle,[-90,90],[-45,45])

        curr_speed = self.car_speed
        
        if (IncreaseTireSpeedInTurns and (Tracked_class !="left_turn")):
            if(angle>30):
                car_speed_turn = interp(angle,[30,45],[80,100])
                curr_speed = car_speed_turn
            elif(angle<-30):
                car_speed_turn = interp(angle,[-45,-30],[100,80])
                curr_speed = car_speed_turn

        
        return angle , curr_speed


    def Obey_LeftTurn(self,Angle,Speed,Mode,Tracked_class):
        
        if (Tracked_class == "left_turn"):
            
            Speed = 50

            if ( (self.prev_Mode_LT =="Detection") and (Mode=="Tracking")):
                self.prev_Mode_LT = "Tracking"
                self.Detected_LeftTurn = True

            elif ( (self.prev_Mode_LT =="Tracking") and (Mode=="Detection")):
                self.Detected_LeftTurn = False
                self.Activat_LeftTurn = True

                if ( ((self.Left_turn_iterations % 20 ) ==0) and (self.Left_turn_iterations>100) ):
                    self.Frozen_Angle = self.Frozen_Angle -7 # Move left by 1 degree 
                if(self.Left_turn_iterations==250):
                    self.prev_Mode_LT = "Detection"
                    self.Activat_LeftTurn = False
                    self.Left_turn_iterations = 0
                self.Left_turn_iterations = self.Left_turn_iterations + 1

                if (self.Activat_LeftTurn or self.Detected_LeftTurn):
                    #Follow previously Saved Route
                    Angle = self.Frozen_Angle

        return Angle,Speed,self.Detected_LeftTurn,self.Activat_LeftTurn


    def OBEY_TrafficLights(self,a,b,Traffic_State,CloseProximity):

        if((Traffic_State == "Stop") and CloseProximity):
            b = 0 # Noob luqman
            self.STOP_MODE_ACTIVATED = True
        else:
            if (self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED):

                if (self.STOP_MODE_ACTIVATED and (Traffic_State=="Go")):
                    self.STOP_MODE_ACTIVATED = False
                    self.GO_MODE_ACTIVATED = True

                elif(self.STOP_MODE_ACTIVATED):
                    b = 0

                elif(self.GO_MODE_ACTIVATED):
                    a = 0.0                    
                    if(self.TrafficLight_iterations==150):
                        self.GO_MODE_ACTIVATED = False
                        print("Interchange Crossed !!!")
                        self.TrafficLight_iterations = 0 #Reset

                    self.TrafficLight_iterations = self.TrafficLight_iterations + 1
        return a,b


    def drive_car(self,Current_State,Inc_TL,Inc_LT):

        [Distance, Curvature, frame_disp , Mode , Tracked_class, Traffic_State, CloseProximity] = Current_State

        current_speed = 0
        
        if((Distance != -1000) and (Curvature != -1000)):

            self.angle_of_car , current_speed = self.follow_Lane(int(frame_disp.shape[1]/4), Distance,Curvature , Mode , Tracked_class )
        
        if Inc_LT:
            self.angle_of_car,current_speed, Detected_LeftTurn, Activat_LeftTurn = self.Obey_LeftTurn(self.angle_of_car,current_speed,Mode,Tracked_class)
        else:
            Detected_LeftTurn = False
            Activat_LeftTurn = False

        if Inc_TL:
            self.angle_of_car,current_speed = self.OBEY_TrafficLights(self.angle_of_car,current_speed,Traffic_State,CloseProximity)        


        return self.angle_of_car,current_speed, Detected_LeftTurn, Activat_LeftTurn 

class Car:
    def __init__( self,Inc_TL = True, Inc_LT = True ):
        
        self.Control_ = Control()
        self.Inc_TL = Inc_TL
        self.Inc_LT = Inc_LT

    def display_state(self,frame_disp,angle_of_car,current_speed,Tracked_class,Traffic_State,Detected_LeftTurn, Activat_LeftTurn):
    
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

        cv2.putText(frame_disp,"Traffic Light State = [ "+Traffic_State+" ] ",(20,60),cv2.FONT_HERSHEY_COMPLEX,0.35,255)
        
        if (Tracked_class=="left_turn"):
            font_Scale = 0.32
            if (Detected_LeftTurn):
                Tracked_class = Tracked_class + " : Detected { True } "
            else:
                Tracked_class = Tracked_class + " : Activated { "+ str(Activat_LeftTurn) + " } "
        else:
            font_Scale = 0.37
        cv2.putText(frame_disp,"Sign Detected ==> "+str(Tracked_class),(20,80),cv2.FONT_HERSHEY_COMPLEX,font_Scale,(0,255,255),1)

    def driveCar(self,frame):
        
        img = frame[0:640,238:1042]
        img = cv2.resize(img,(320,240))

        img_orig = img.copy()

        if self.Inc_TL:
            Traffic_State, CloseProximity = detect_TrafficLights(img_orig.copy())
        else:
            Traffic_State = "Unknown"
            CloseProximity = False

        distance, Curvature = detect_Lane(img)
        
        Mode , Tracked_class = detect_Signs(img_orig,img)

        Current_State = [distance, Curvature, img, Mode, Tracked_class, Traffic_State, CloseProximity]

        Angle,Speed, Detected_LeftTurn, Activat_LeftTurn  = self.Control_.drive_car(Current_State,self.Inc_TL,self.Inc_LT)

        self.display_state(img,Angle,Speed,Tracked_class,Traffic_State, Detected_LeftTurn, Activat_LeftTurn)

        # Translate [ Real World angle and speed ===>> ROS Car Control Range ]
        Angle=interp(Angle,[-45,45],[0.5,-0.5])
        if (Speed!=0):
            Speed=interp(Speed,[30,90],[1,2])

        Speed = float(Speed)

        return Angle, Speed, img