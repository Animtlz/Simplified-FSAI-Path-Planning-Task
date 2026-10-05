from __future__ import annotations

from typing import List
import math
from src.models import CarPose, Cone, Path2D

blue = 1
yellow = 0
track_width = 2

class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones


    def get_cones(self,color): # 1 blue 0 yellow
        color_cones = []
        for cone in self.cones:
            if(cone.color == color):
                color_cones.append(cone)
        return color_cones


    def distance_to_car(self,x,y):
        return ((x-self.car_pose.x,y-self.car_pose.y,((x-self.car_pose.x)**2+(y-self.car_pose.y)**2)**0.5)) 
    
    def sort_cones(self,cone_list):
        for i in range (len(cone_list)-1):
            for j in range(len(cone_list)-1):
                if(self.distance_to_car(cone_list[j].x,cone_list[j].y)[0] > self.distance_to_car(cone_list[j+1].x,cone_list[j+1].y)[0]):
                    cone_list[j],cone_list[j+1] = cone_list[j+1],cone_list[j]
                elif ((self.distance_to_car(cone_list[j].x,cone_list[j].y)[0] == self.distance_to_car(cone_list[j+1].x,cone_list[j+1].y)[0]) and (self.distance_to_car(cone_list[j].x,cone_list[j].y)[2] > self.distance_to_car(cone_list[j+1].x,cone_list[j+1].y)[2])):
                    cone_list[j],cone_list[j+1] = cone_list[j+1],cone_list[j]
        return cone_list

    def sort_midpoints(self,midpoint_list):
        for i in range (len(midpoint_list)-1):
            for j in range(len(midpoint_list)-1):
                if(self.distance_to_car(midpoint_list[j][0],midpoint_list[j][1])[0] > self.distance_to_car(midpoint_list[j+1][0],midpoint_list[j+1][1])[0]):
                    midpoint_list[j],midpoint_list[j+1] = midpoint_list[j+1],midpoint_list[j]
                elif ((self.distance_to_car(midpoint_list[j][0],midpoint_list[j][1])[0] == self.distance_to_car(midpoint_list[j+1][0],midpoint_list[j+1][1])[0]) and (self.distance_to_car(midpoint_list[j][0],midpoint_list[j][1])[2] > self.distance_to_car(midpoint_list[j+1][0],midpoint_list[j+1][1])[2])):
                    midpoint_list[j],midpoint_list[j+1] = midpoint_list[j+1],midpoint_list[j]

        return midpoint_list
    
    def matching(self,match_cone):
        matching_pair = []
        for cone in self.cones:
            if((cone.x == match_cone.x or cone.y == match_cone.y) and cone.color != match_cone.color):
                return (True,cone)
        return (False,None)

    def get_all_cone_pairs(self):
        cone_pairs = []
        for cone in self.cones:
            if(self.matching(cone)[0]):
                cone_pairs.append((cone,self.matching(cone)[1]))
        return list(dict.fromkeys(cone_pairs))
    
    def get_any_cone_pair(self):
        for cone in self.cones:
            if(self.matching(cone)[0]):
                return (cone,self.matching(cone)[1])
        return None

    def get_shift_values(self,cone,cone_list,track_width):
        shift_values = [0,0]
        cone_index = cone_list.index(cone)
        adjacent_point = (0,0)
        if(len(cone_list) == 1):
            adjacent_point = (self.car_pose.x,self.car_pose.y) 
        elif(cone_index+1 < len(cone_list)):
            adjacent_point = (cone_list[cone_index+1].x,cone_list[cone_index+1].y)
        elif(cone_index-1 >= 0 ):
            adjacent_point = (cone_list[cone_index-1].x,cone_list[cone_index-1].y)

        if(cone.color == blue):
            if(cone.y == adjacent_point[1]):
                shift_values[1] = -track_width/2
            elif(cone.x == adjacent_point[0]):
                shift_values[0] = track_width/2
            else:
                shift_values = [0,-track_width/2]
        else:
            if(cone.y == adjacent_point[1]):
                shift_values[1] = track_width/2
            elif(cone.x == adjacent_point[0]):
                shift_values[0] = -track_width/2
            else:
                shift_values = [0,track_width/2]
        return shift_values
    
    def get_lonely_midpoint(self,lonely_cone,width = track_width):
        blue_cones = self.sort_cones(self.get_cones(blue))
        yellow_cones = self.sort_cones(self.get_cones(yellow))
        shift_values = []
        if(lonely_cone.color == blue):
            shift_values = self.get_shift_values(lonely_cone,blue_cones,width)
        else:
            shift_values = self.get_shift_values(lonely_cone,yellow_cones,width)
        return (lonely_cone.x + shift_values[0],lonely_cone.y + shift_values[1])

    def get_lonely_midpoints(self,lonely_cones,width = track_width):
        lonely_midpoints = []
        for lonely_cone in lonely_cones:
            lonely_midpoints.append(self.get_lonely_midpoint(lonely_cone,width))
        return lonely_midpoints
    
    def get_lonely_cones(self):
        lonely_cones = []
        if self.cones:
            for cone in self.cones:
                if(not self.matching(cone)[0]):
                    lonely_cones.append(cone)
        return lonely_cones

    def get_midpoints(self):
        cone_pairs = self.get_all_cone_pairs()
        midpoints = []
        lonely_cones = self.sort_cones(self.get_lonely_cones())
        for cone_pair in cone_pairs:
            midpoints.append(((cone_pair[0].x+cone_pair[1].x)/2,(cone_pair[0].y+cone_pair[1].y)/2))

        if(len(self.cones) == len(lonely_cones)):
                midpoints.extend(self.get_lonely_midpoints(lonely_cones))
        else: 
            pair = self.get_any_cone_pair()
            scenario_track_width = ((pair[0].x - pair[1].x)**2 + (pair[0].y-pair[1].y)**2)**0.5
            midpoints.extend(self.get_lonely_midpoints(lonely_cones,scenario_track_width))
        
        # if(len(blue_cones) == len(yellow_cones)):                    
        #     for i in range(len(blue_cones)):
        #         midpoints.append(((blue_cones[i].x + yellow_cones[i].x)/2,(blue_cones[i].y + yellow_cones[i].y)/2))
        # else:
            # if(blue_cones and yellow_cones):
            #     for i in range(len(blue_cones) if len(blue_cones)>len(yellow_cones) else len(yellow_cones)):
            #         if(yellow_cones[i].x != blue_cones[i].x and yellow_cones[i].y != blue_cones[i]):
            #             if(len(blue_cones)>len(yellow_cones)):
            

            # if(len(self.cones) == len(lonely_cones)):
            #         midpoints.extend(self.get_lonely_midpoints(lonely_cones))
            # else: 
            #     pair = self.get_any_cone_pair()
            #     scenario_track_width = ((pair[0].x - pair[1].x)**2 + (pair[0].y-pair[1].y)**2)**0.5
            #     midpoints.extend(self.get_lonely_midpoints(lonely_cones))

                    
        return self.sort_midpoints(midpoints)

    def follow_point(self,point, path):
        sp = (self.car_pose.x,self.car_pose.y)

        if path : 
            sp = path[-1]

        target_angle = math.atan2(point[1]-sp[1],point[0]-sp[0])
        target_dist = math.hypot(abs(sp[1]-point[1]),abs(sp[0]-point[0]))

        step = 0.05
        max_steps = 10/step
        taken_steps = 0
        cp = list(sp)
        while(taken_steps < max_steps and  taken_steps < target_dist/step):
            cp[0]+= step*math.cos(target_angle)
            cp[1]+= step*math.sin(target_angle)
            path.append((cp[0],cp[1]))
            taken_steps+=1

        dx = path[-1][0] - path[-2][0]
        dy = path[-1][1] - path[-2][1]
        target_angle = math.atan2(dy,dx)


        # while(taken_steps < max_steps):
        #     cp[0] += step * math.cos(target_angle)
        #     cp[1] += step * math.sin(target_angle)
        #     path.append((cp[0],cp[1]))
        #     taken_steps+=1

    
    def generatePath(self) -> Path2D:
        midpoints = self.get_midpoints()

        path = []

        if midpoints:
            for midpoint in midpoints:
                self.follow_point(midpoint,path)
        else:
            target = (self.car_pose.x + 7*math.cos(self.car_pose.yaw),self.car_pose.y + 7*math.sin(self.car_pose.yaw))
            self.follow_point(target,path)
        return path