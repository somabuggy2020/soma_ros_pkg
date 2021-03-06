#!/usr/bin/env python

'''
Module for providing the Forest Environment
'''
import logging as log
import numpy as np
import random as rnd
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import matplotlib.patches as patches

#plot or not
isShow = True

log.basicConfig(level=log.DEBUG, format='[%(module)s:%(funcName)s(%(lineno)s)] %(message)s')

class Environment:
    def __init__(self,):
        self.TreeN = 0
        self.Trees = []

        self.fig = plt.figure(figsize=(4,4))
        self.fig.canvas.set_window_title('Tree Map')
        self.ax = plt.subplot(1,1,1)
        self.ax.set_aspect('equal')

    def make_random(self,TreeN,xrange,yrange,rrange):
        log.info('TreeN {}'.format(TreeN))
        self.TreeN = TreeN

        while len(self.Trees) < self.TreeN:
            xi = rnd.uniform(xrange[0],xrange[1])
            yi = rnd.uniform(yrange[0],yrange[1])
            ri = rnd.uniform(rrange[0],rrange[1])
            
            #resampling if distance from neighbor tree less than 2.5m
            if len(self.Trees) >= 1:
                #make KD tree
                kdTree = KDTree([[ti[0],ti[1]] for ti in self.Trees])
                d,j = kdTree.query([xi,yi])
                if d <= 2.5:
                    continue
            
            self.Trees.append([xi,yi,ri])

        self.Trees = [[round(ti[0],2),round(ti[1],2),round(ti[2],2)] for ti in self.Trees]

        self.ax.set_xlim(xrange[0],xrange[1])
        self.ax.set_ylim(yrange[0],yrange[1])
        return None

    def make_from(self,):
        log.info('')
        return None

    def show(self,):
        for x,y,r in self.Trees:
            c = patches.Circle(xy=(x,y),radius=r,color='g')
            self.ax.add_patch(c)
        
        plt.show()


if __name__=='__main__':
    print('Test Environemnt')

    env = Environment()
    env.make_random(
        TreeN=10,
        xrange=[0.0, 15.0], 
        yrange=[0.0, 15.0], 
        rrange=[0.1, 0.8])

    print('Tree N:',env.TreeN)    
    print('[x,y,r]=',env.Trees)

    if isShow:
        env.show()



