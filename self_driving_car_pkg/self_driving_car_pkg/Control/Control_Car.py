from numpy import interp
import cv2

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
                print("Left Detected")
                print("self.Detected_LeftTurn ",self.Detected_LeftTurn)
                self.Detected_LeftTurn = True
                print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Tracking <<<<<<<<<<<<<<<<<<<<<<<<<<<")
            elif ( (self.prev_Mode_LT =="Tracking") and (Mode=="Detection")):
                print("Left Activated")
                self.Detected_LeftTurn = False
                self.Activat_LeftTurn = True
                print("self.Activat_LeftTurn ",self.Activat_LeftTurn)

                if ( ((self.Left_turn_iterations % 20 ) ==0) and (self.Left_turn_iterations>100) ):
                    self.Frozen_Angle = self.Frozen_Angle -7 # Move left by 1 degree 
                if(self.Left_turn_iterations==250):
                    print("Left DeActivated")
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
            print("**************STOP MODE ACTIVATED !!!!!**************")
            b = 0 # Noob luqman
            self.STOP_MODE_ACTIVATED = True
        else:
            if (self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED):

                if (self.STOP_MODE_ACTIVATED and (Traffic_State=="Go")):
                    print("**************GO MODE ACTIVATED !!!!!**************")
                    self.STOP_MODE_ACTIVATED = False
                    self.GO_MODE_ACTIVATED = True

                elif(self.STOP_MODE_ACTIVATED):
                    print("**************STOP MODE EXECUTING !!!!!**************")
                    b = 0

                elif(self.GO_MODE_ACTIVATED):
                    print("**************GO MODE EXECUTING !!!!!**************")
                    a = 0.0                    
                    if(self.TrafficLight_iterations==150):
                        self.GO_MODE_ACTIVATED = False
                        print("Interchange Crossed !!!")
                        self.TrafficLight_iterations = 0 #Reset

                    self.TrafficLight_iterations = self.TrafficLight_iterations + 1
        return a,b




    def Drive_Car(self,Current_State):

        [Distance, Curvature, frame_disp , Mode , Tracked_class, Traffic_State, CloseProximity] = Current_State

        current_speed = 0
        
        if((Distance != -1000) and (Curvature != -1000)):

            self.angle_of_car , current_speed = self.follow_Lane(int(frame_disp.shape[1]/4), Distance,Curvature , Mode , Tracked_class )
        
        
        self.angle_of_car,current_speed, Detected_LeftTurn, Activat_LeftTurn = self.Obey_LeftTurn(self.angle_of_car,current_speed,Mode,Tracked_class)

        self.angle_of_car,current_speed = self.OBEY_TrafficLights(self.angle_of_car,current_speed,Traffic_State,CloseProximity)        


        return self.angle_of_car,current_speed, Detected_LeftTurn, Activat_LeftTurn 