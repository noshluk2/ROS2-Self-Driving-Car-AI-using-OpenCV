
import cv2
# GPS-Navigation
from .GPS_Navigation.bot_localization import bot_localizer
from .GPS_Navigation.bot_mapping import bot_mapper
from .GPS_Navigation.bot_pathplanning import bot_pathplanner
from .GPS_Navigation.bot_motionplanning import bot_motionplanner
class Navigator():
    
    def __init__(self):
        
        # Creating objects for each stage of the robot navigation
        self.bot_localizer = bot_localizer()
        self.bot_mapper = bot_mapper()
        self.bot_pathplanner = bot_pathplanner()
        self.bot_motionplanner = bot_motionplanner()
    
    cv2.namedWindow("SatView (Live)",cv2.WINDOW_NORMAL)
    cv2.namedWindow("SatView (Occupancy Grid)",cv2.WINDOW_NORMAL)


    def navigate_to_home(self,sat_view):
        cv2.imshow("SatView (Live)", sat_view)
        cv2.waitKey(0)
        # Creating frame to display current robot state to user        
        frame_disp = sat_view.copy()
        
        # [Stage 1: Localization] Localizing robot at each iteration        
        self.bot_localizer.localize_bot(sat_view, frame_disp)
        cv2.imshow("SatView (Occupancy Grid)", self.bot_localizer.maze_og)
        cv2.waitKey(0)
        # [Stage 2: Mapping] Converting Image to Graph
        self.bot_mapper.graphify(self.bot_localizer.maze_og)

        # [Stage 3: PathPlanning] Using {User Specified PathPlanner} to find path to goal        
        start = self.bot_mapper.Graph.start
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze

        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="dijisktra")
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star")
        if config.debug and config.debug_pathplanning:
            print("\nNodes Visited [Dijisktra V A-Star*] = [ {} V {} ]".format(self.bot_pathplanner.dijisktra.dijiktra_nodes_visited,self.bot_pathplanner.astar.astar_nodes_visited))


        # [Stage 4: MotionPlanning] Reach the (maze exit) by navigating the path previously computed
        bot_loc = self.bot_localizer.loc_car
        path = self.bot_pathplanner.path_to_goal
        self.bot_motionplanner.nav_path(bot_loc, path, self.vel_msg, self.velocity_publisher)

        # Displaying bot solving maze  (Live)
        img_shortest_path = self.bot_pathplanner.img_shortest_path
        self.bot_motionplanner.display_control_mechanism_in_action(bot_loc, path, img_shortest_path, self.bot_localizer, frame_disp)
        
        # View bot view on left to frame Display
        bot_view = cv2.resize(self.bot_view, (int(frame_disp.shape[0]/2),int(frame_disp.shape[1]/2)))
        bot_view = bot_view[0:int(bot_view.shape[0]/1.5),:]

        cv2.imshow("SatView (Live)", frame_disp)
        cv2.waitKey(1)