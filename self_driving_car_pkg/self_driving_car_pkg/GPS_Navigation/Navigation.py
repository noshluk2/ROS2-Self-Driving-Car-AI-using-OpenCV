'''
> Purpose :
Node to perform the actual (worthy of your time) task of maze solving ;) 
- Robot velocity interface
- Upper Video camera as well

> Usage :
You need to write below command in terminal where your pacakge is sourced
- ros2 run maze_bot maze_solver

Note : Name of the node is actually name of executable file described in setup.py file of our package and not the name of python file

> Inputs:
This node is subscribing video feed from (Satellite or DroneCam)

> Outputs:
This node publishes on topic "/cmd_vel" , the required velocity ( linear and angular ) to move the robot

Author :
Haider Abbasi

Date :
18/03/22
'''


import cv2

from .bot_localization import bot_localizer
# from .bot_mapping import bot_mapper
# from .bot_pathplanning import bot_pathplanner
# from .bot_motionplanning import bot_motionplanner

from .utilities import Debugging
from . import config

class Navigator():

    def __init__(self):
        
        # Creating objects for each stage of the robot navigation
        self.bot_localizer = bot_localizer()
        # self.bot_mapper = bot_mapper()
        # self.bot_pathplanner = bot_pathplanner()
        # self.bot_motionplanner = bot_motionplanner()

        self.debugging = Debugging()



    def navigate_to_home(self,sat_view):
        
        self.debugging.setDebugParameters()

        # Creating frame to display current robot state to user        
        frame_disp = sat_view.copy()
        
        # [Stage 1: Localization] Localizing robot at each iteration        
        self.bot_localizer.localize_bot(sat_view, frame_disp)

        # # [Stage 2: Mapping] Converting Image to Graph
        # self.bot_mapper.graphify(self.bot_localizer.maze_og)

        # # [Stage 3: PathPlanning] Using {User Specified PathPlanner} to find path to goal        
        # start = self.bot_mapper.Graph.start
        # end = self.bot_mapper.Graph.end
        # maze = self.bot_mapper.maze

        # self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="dijisktra")
        # self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star")
        # if config.debug and config.debug_pathplanning:
        #     print("\nNodes Visited [Dijisktra V A-Star*] = [ {} V {} ]".format(self.bot_pathplanner.dijisktra.dijiktra_nodes_visited,self.bot_pathplanner.astar.astar_nodes_visited))


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

        # # Draw & Display [For better Understanding of current robot state]
        # center_frame_disp = int(frame_disp.shape[0]/2)
        # center_bot_view = int(bot_view.shape[0]/2)
        # bot_offset = center_frame_disp - center_bot_view
        # center_img_shortest_path = int(img_shortest_path.shape[0]/2)
        # isp_offset = center_frame_disp - center_img_shortest_path

        # bot_view = self.draw_bot_speedo(bot_view,self.bot_motionplanner.curr_speed)

        # if config.debug_live:
        #     self.overlay_live(frame_disp,img_shortest_path,self.bot_mapper.maze_interestPts,self.bot_pathplanner.choosen_route)

        # orig_col = 40 + int(bot_view.shape[1]/4)
        # orig = (orig_col,bot_offset-10)
        # cv2.putText(frame_disp, "Bot View", orig, cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0),3)
        # frame_disp = cv2.rectangle(frame_disp, (20,bot_offset), (bot_view.shape[1]+20,(bot_view.shape[0]+bot_offset)), (0,0,255),12)
        # frame_disp[bot_offset:(bot_view.shape[0]+bot_offset),20:bot_view.shape[1]+20] = bot_view
 
        cv2.imshow("Maze (Live)", frame_disp)
        cv2.waitKey(1)

