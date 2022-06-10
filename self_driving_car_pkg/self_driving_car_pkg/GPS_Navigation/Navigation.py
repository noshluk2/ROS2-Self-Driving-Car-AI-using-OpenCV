
import cv2
# GPS-Navigation
from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
from .bot_pathplanning import bot_pathplanner
from .bot_motionplanning import bot_motionplanner

import sys
from .utilities import find_point_in_FOR,Debugging
from . import config

class Navigator():
    
    def __init__(self):

        self.debugging = Debugging()

        # Creating objects for each stage of the robot navigation
        self.bot_localizer = bot_localizer()
        self.bot_mapper = bot_mapper()
        self.bot_pathplanner = bot_pathplanner()
        self.bot_motionplanner = bot_motionplanner()
    
        self.accquiring_destination = True
        self.destination = []
    
    cv2.namedWindow("SatView (Live)",cv2.WINDOW_NORMAL)

    # function to display the coordinates of
    # of the points clicked on the image
    def click_event(self,event, x, y, flags, params):

        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:

            # displaying the coordinates
            # on the Shell
            print(x, ' ', y)

            self.destination = (x,y)

    def navigate_to_home(self,sat_view):

        self.debugging.setDebugParameters()
        
        # Creating frame to display current robot state to user        
        frame_disp = sat_view.copy()
        
        # [Stage 1: Localization] Localizing robot at each iteration        
        self.bot_localizer.localize_bot(sat_view, frame_disp)

        if self.accquiring_destination:
            cv2.imshow("Mark your destination!!!",sat_view)
            cv2.setMouseCallback("Mark your destination!!!", self.click_event)
            while(self.destination==[]):
                cv2.waitKey(1)
            if self.destination!=[]:
                cv2.destroyWindow("Mark your destination!!!")
                self.accquiring_destination = False
                self.destination = find_point_in_FOR(self.destination,self.bot_localizer.transform_arr,self.bot_localizer.rot_mat,sat_view.shape[1],sat_view.shape[0])
            else:
                print("Destination not specified.... Exiting!!!")
                sys.exit()

        # [Stage 2: Mapping] Converting Image to Graph
        self.bot_mapper.graphify(self.bot_localizer.maze_og,self.bot_localizer.loc_car,self.destination)

        # [Stage 3: PathPlanning] Using {User Specified PathPlanner} to find path to goal        
        start = self.bot_mapper.Graph.start
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze

        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="dijisktra")
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star")
        if config.debug and config.debug_pathplanning:
            print("\nNodes Visited [Dijisktra V A-Star*] = [ {} V {} ]".format(self.bot_pathplanner.dijisktra.dijiktra_nodes_visited,self.bot_pathplanner.astar.astar_nodes_visited))

        # # [Stage 4: MotionPlanning] Reach the (maze exit) by navigating the path previously computed
        # bot_loc = self.bot_localizer.loc_car
        # path = self.bot_pathplanner.path_to_goal
        # self.bot_motionplanner.nav_path(bot_loc, path, self.vel_msg, self.velocity_publisher)

        # # Displaying bot solving maze  (Live)
        # img_shortest_path = self.bot_pathplanner.img_shortest_path
        # self.bot_motionplanner.display_control_mechanism_in_action(bot_loc, path, img_shortest_path, self.bot_localizer, frame_disp)
        
        # # View bot view on left to frame Display
        # bot_view = cv2.resize(self.bot_view, (int(frame_disp.shape[0]/2),int(frame_disp.shape[1]/2)))
        # bot_view = bot_view[0:int(bot_view.shape[0]/1.5),:]

        cv2.imshow("SatView (Live)", frame_disp)
        cv2.waitKey(1)