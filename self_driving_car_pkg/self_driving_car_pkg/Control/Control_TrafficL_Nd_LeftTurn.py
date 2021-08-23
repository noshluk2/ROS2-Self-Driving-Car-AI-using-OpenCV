
class TL_Control:
    def __init__(self):
        TrafficLight_iterations = 0
        Prev_Turn_angle = 1.5
        GO_MODE_ACTIVATED = False
        STOP_MODE_ACTIVATED = False

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
                    self.Prev_Turn_angle = a # Save Computed Turn Angle

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
