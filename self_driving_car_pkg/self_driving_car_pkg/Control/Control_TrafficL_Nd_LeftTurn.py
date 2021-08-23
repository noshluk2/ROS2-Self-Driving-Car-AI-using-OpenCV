from ..config import config
class Control:
    def __init__(self):
        self.TrafficLight_iterations = 0
        self.Prev_Turn_angle = 1.5
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False

        self.prev_Mode = "Detection"
        self.Left_turn_iterations = 0
        self.Frozen_distance = 0
        self.Frozen_Curvature = 0

    def Obey_LeftTurn(self,distance,Curvature,Mode,Tracked_class):
        
        if ( (self.prev_Mode =="Detection") and (Mode=="Tracking") and (Tracked_class=="left_turn") ):
            self.prev_Mode = "Tracking"
            config.Activat_LeftTurn = True
            self.Frozen_distance = distance
            self.Frozen_Curvature = Curvature
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Tracking <<<<<<<<<<<<<<<<<<<<<<<<<<<")
        elif ( (self.prev_Mode =="Tracking") and (Mode=="Detection") and (Tracked_class=="left_turn") ):
            print("Left Activated")
            print("config.Activat_LeftTurn ",config.Activat_LeftTurn)
            if ( ((self.Left_turn_iterations % 24 ) ==0) and (self.Left_turn_iterations>25) ):
                self.Frozen_Curvature = self.Frozen_Curvature -1 # Move left by 1 degree 
            if(self.Left_turn_iterations==100):
                print("Left DeActivated")
                self.prev_Mode = "Detection"
                config.Activat_LeftTurn = False
                self.Left_turn_iterations = 0
            self.Left_turn_iterations = self.Left_turn_iterations + 1
            if (config.Activat_LeftTurn == True):
                distance = self.Frozen_distance
                Curvature = self.Frozen_Curvature

        return distance,Curvature


    def OBEY_TrafficLights(self,a,b,Traffic_State,CloseProximity):

        if((Traffic_State == "Stop") and CloseProximity):
            print("**************STOP MODE ACTIVATED !!!!!**************")
            b = 0.0 # Noob luqman
            self.STOP_MODE_ACTIVATED = True
        else:
            if (self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED):

                if (self.STOP_MODE_ACTIVATED and (Traffic_State=="Go")):
                    print("**************GO MODE ACTIVATED !!!!!**************")
                    self.STOP_MODE_ACTIVATED = False
                    self.GO_MODE_ACTIVATED = True
                    self.Prev_Turn_angle = 0.0 # Save Computed Turn Angle
                    #self.Prev_Turn_angle = a # Save Computed Turn Angle

                elif(self.STOP_MODE_ACTIVATED):
                    print("**************STOP MODE EXECUTING !!!!!**************")
                    b = 0.0

                elif(self.GO_MODE_ACTIVATED):
                    print("**************GO MODE EXECUTING !!!!!**************")
                    a = self.Prev_Turn_angle                    
                    if(self.TrafficLight_iterations==100):
                        self.GO_MODE_ACTIVATED = False
                        print("Interchange Crossed !!!")
                        self.TrafficLight_iterations = 0 #Reset

                    self.TrafficLight_iterations = self.TrafficLight_iterations + 1
        return a,b
