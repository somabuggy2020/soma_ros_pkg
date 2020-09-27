#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx #graph structure module
from scipy.spatial import Delaunay
from scipy.spatial import distance
import itertools

from Environment import Environment

'''
Graph Based Path Planner module

input: Environment instance
output: 2-D way point list
'''

isShow = True

class GraphBasedPathPlanner:
    def __init__(self,):


        self.fig = plt.figure(figsize=(4,4))
        self.ax = plt.subplot(2,1,1)
        self.ax.set_aspect('equal')

    def planning(self, env):
        if len(env.Trees)<=2:
            print('Not enough landmarks')
            print('{} <= 2'.format(len(env.Trees)))




# module test
if __name__=='__main__':
    print('Test Graph Based Path Planner')

    env = Environment()
    gbpp  = GraphBasedPathPlanner()

    #make random placed trees forest
    env.make_random(
        TreeN=15,
        xrange=[0.0, 30.0],
        yrange=[0.0,30.0],
        rrange=[0.1,0.8]
    )

    print('Tree N:',env.TreeN)    
    print('Trees:',env.Trees)

    gbpp.planning(env)

    if isShow:
        env.show()
        
