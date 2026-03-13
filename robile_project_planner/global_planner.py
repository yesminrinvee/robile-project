import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import math

class AStarPlanner:
    def __init__(self, map_data: OccupancyGrid):
        self.map = map_data
        self.width = map_data.info.width
        self.height = map_data.info.height
        self.resolution = map_data.info.resolution
        self.origin = map_data.info.origin.position
        
    def world_to_map(self, x, y):
        """Convert world coordinates to map indices"""
        mx = int((x - self.origin.x) / self.resolution)
        my = int((y - self.origin.y) / self.resolution)
        return mx, my
    
    def map_to_world(self, mx, my):
        """Convert map indices to world coordinates"""
        x = mx * self.resolution + self.origin.x
        y = my * self.resolution + self.origin.y
        return x, y
    
    def is_obstacle(self, mx, my):
        """Check if map cell is an obstacle"""
        if mx < 0 or mx >= self.width or my < 0 or my >= self.height:
            return True
        index = my * self.width + mx
        return self.map.data[index] > 50  # Occupied if > 50
    
    def plan(self, start_x, start_y, goal_x, goal_y):
        """A* path planning"""
        start_mx, start_my = self.world_to_map(start_x, start_y)
        goal_mx, goal_my = self.world_to_map(goal_x, goal_y)
        
        # A* implementation (simplified - you'll need to complete this)
        # Returns list of (x, y) world coordinates
        open_set = {start_mx, start_my}
        came_from = {}
        g_score = {(start_mx, start_my): 0}
        f_score = {(start_mx, start_my): self.heuristic(start_mx, start_my, goal_mx, goal_my)}
        
        # ... A* algorithm implementation ...
        
        # After finding path, extract waypoints
        path = self.reconstruct_path(came_from, current)
        waypoints = self.extract_waypoints(path)
        
        return waypoints
    
    def heuristic(self, mx1, my1, mx2, my2):
        """Manhattan distance heuristic"""
        return abs(mx1 - mx2) + abs(my1 - my2)
    
    def extract_waypoints(self, path, step=5):
        """Extract waypoints every 'step' cells"""
        waypoints = []
        for i, (mx, my) in enumerate(path):
            if i % step == 0:
                x, y = self.map_to_world(mx, my)
                waypoints.append((x, y))
        return waypoints
