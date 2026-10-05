from __future__ import annotations

from math import ceil, hypot ,cos,sin
from typing import List

from src.models import CarPose, Cone, Path2D
HALF_W=1.5 # i assumed the minimum track width to be 3 meters
Max_DIFFERENCE=5 # i assumed the maximum distance between two cones to be 5 meters
straight_ahead_distance=5 # i assumed the distance to be 5 meters if no cones are detected
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
    def get_midpoint(self,cone1,cone2):
        x= (cone1.x+cone2.x)/2
        y= (cone1.y+cone2.y)/2
        return (x, y)
    
    def get_distance(self,cone):
        distance=(cone.x-self.car_pose.x)*cos(self.car_pose.yaw)+(cone.y-self.car_pose.y)*sin(self.car_pose.yaw)
        return distance
    def get_virtual_center(self,cone,vx,vy,half_w=HALF_W):
        length=hypot(vx,vy)   #vx=con1.x-cone2.x,vy=cone1.y-cone2.y
        if length==0:      #slope =vy/vx , penslope=-vx/vy
            print("length is zero")
            return 
        vx=vx/length #normilize  into unit vector
        vy=vy/length
        if cone.color==1: #blue
            px=vy
            py=-vx
        else: #yellow
            px=-vy
            py=vx
        return (cone.x+px*half_w,cone.y+py*half_w)
    def sort_cones(self,cones):
        #shifted_cones =List[Cone]
        #k=0
        #for cone in cones:
        #    shifted_cones[k].x=cone.x-self.car_pose.x
        #    shifted_cones[k].y=cone.y-self.car_pose.y
        #    k+=1
        #sorted_cones=sorted(shifted_cones,key=lambda cone:)
        withempty_cones=[cone for cone in cones if self.get_distance(cone)>0]
        cones=sorted(withempty_cones,key=lambda cone: self.get_distance(cone))
        #for cone in cones:
            #if self.get_distance(cone)<0:
               # cones.remove(cone)
        return cones
        
    def get_pairs(self,right_cones,left_cones,max_difference=Max_DIFFERENCE):
        pairs,remainingL=[],[]
        remainingR=list(right_cones)
        for l in left_cones:
            best,best_diff=None,max_difference
            for r in remainingR:
                dis=hypot(l.x-r.x,l.y-r.y)
                if dis<best_diff:
                    best,best_diff=r,dis
            if best is not None:
                pairs.append((l,best))
                remainingR.remove(best)
            else:
                remainingL.append(l)
        return pairs ,remainingL,remainingR

    def if_noCorsCones(self,cones):
        cones = sorted(cones, key=lambda c: hypot(c.x - self.car_pose.x, c.y - self.car_pose.y))
        if len(cones)==1:
            return [self.get_virtual_center(cones[0],cos(self.car_pose.yaw),sin(self.car_pose.yaw))] #car pose is the only information we have for one cone
        pts = []
        for i, cone in enumerate(cones):
            if i < len(cones) - 1: #before last cone 
                next = cones[i + 1]
                vx, vy = next.x - cone.x, next.y - cone.y
            else:
                prev = cones[i - 1] #last cone, use previous cone to get the direction
                vx, vy = cone.x - prev.x, cone.y - prev.y
            
            pts.append(self.get_virtual_center(cone, vx, vy))
        return pts
    def add_points(self,points,step=0.5):
        allp=[(self.car_pose.x,self.car_pose.y)]+points #start with the car origin and add cone midpoints
        path=[allp[0]]# add the car orging first to path
        for i in range(len(allp)-1):
            p1=allp[i] 
            p2=allp[i+1]
            distance=hypot(p2[0]-p1[0],p2[1]-p1[1]) #get the distance so we can divide it and see how many points we need
            num_steps=max(1,ceil(distance/step))#get at lest one step
            for j in range(1,num_steps+1):
                partstep=j/num_steps
                x=p1[0]+(p2[0]-p1[0])*partstep
                y=p1[1]+(p2[1]-p1[1])*partstep
                path.append((x,y))
        return path
    def continue_path(self,path,step=0.5,max_distance=straight_ahead_distance):
        (fx,fy)=path[-1]
        (px,py)=path[-2]
        vx,vy=fx-px,fy-py
        d=hypot(vx,vy)
        if d==0:
            penx,peny=cos(self.car_pose.yaw),sin(self.car_pose.yaw)
        else:
            penx,peny=vx/d,vy/d
        while hypot(fx-self.car_pose.x,fy-self.car_pose.y)<max_distance:
            fx+=penx*step
            fy+=peny*step
            path.append((fx,fy))
        return path


    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.
        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.
        Replace the placeholder implementation below with your algorithm.
        """
        
        #if(len(self.cones)<2):
           # return[]
        cones=self.sort_cones(self.cones)
        left_cones=[cone  for cone in cones if cone.color==1]
        right_cones=[cone for cone in cones if cone.color==0]
        pairs,remainingL,remainingR=self.get_pairs(right_cones,left_cones)
        midpoints=[self.get_midpoint(l,r) for l,r in pairs]
        if len(remainingL)>0:
            midpoints+=self.if_noCorsCones(remainingL)
        if len(remainingR)>0:
            midpoints+=self.if_noCorsCones(remainingR)
        if len(midpoints)==0:
            midpoints.append((self.car_pose.x+straight_ahead_distance*cos(self.car_pose.yaw), self.car_pose.y+straight_ahead_distance*sin(self.car_pose.yaw)))
        midpoints = sorted(midpoints, key=lambda p: hypot(p[0]-self.car_pose.x, p[1]-self.car_pose.y))
        path:Path2D=[]
        path=midpoints 
        pathF =self.add_points(path)
        return self.continue_path(pathF)
        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.


        """
        num_points = 25
        step = 0.5
        cx = self.car_pose.x
        cy = self.car_pose.y
        import math

        path: Path2D = []
        for i in range(1, num_points + 1):
            dx = math.cos(self.car_pose.yaw) * step * i
            dy = math.sin(self.car_pose.yaw) * step * i
            path.append((cx + dx, cy + dy))
        """
        
        return path
