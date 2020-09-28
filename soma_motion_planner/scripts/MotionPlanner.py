#!/usr/bin/env python
from scipy.spatial import KDTree
from Environment import Environment
from GraphBasedPathPlanner import GraphBasedPathPlanner

class MotionPlanner:
    def __init__(self,):
        pass

    def planning(self,waypoints):
        pass

if __name__=='__main__':
    print('Test Travelling Motion Planner')

    env = Environment()
    gbpp  = GraphBasedPathPlanner()
    motionPlanner = MotionPlanner()

    #make random placed trees forest
    env.make_random(
        TreeN=15,
        xrange=[0.0, 30.0],
        yrange=[0.0,30.0],
        rrange=[0.1,0.8]
    )

    waypoints = gbpp.planning(env)


