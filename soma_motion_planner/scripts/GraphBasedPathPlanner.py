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
        self.landmarks = []

        self.fig = plt.figure(figsize=(8,8))
        self.ax_G = plt.subplot(2,2,1)
        self.ax_Ge = plt.subplot(2,2,2)
        # self.ax_G.set_aspect('equal')

    def DelaunayNetwork(self,):
        pos = [(p[0],p[1]) for p in self.landmarks]

        #Delaunay Triangle Deconposition
        tri = Delaunay(pos)

        #new Graph object
        G = nx.Graph()

        for t in tri.simplices: #triangle iteration
            if  distance.euclidean(pos[1], pos[0]) >= 2.5:
                # G.add_edge(t[0],t[1],weight=distance.euclidean(pos[1], pos[0]))
                G.add_edge(t[0],t[1])
            if distance.euclidean(pos[2], pos[1]) >= 2.5:
                # G.add_edge(t[1],t[2],weight=distance.euclidean(pos[2], pos[1]))
                G.add_edge(t[1],t[2])
            if distance.euclidean(pos[0], pos[2]) >= 2.5:
                # G.add_edge(t[2],t[0],weight=distance.euclidean(pos[0], pos[2]))
                G.add_edge(t[2],t[0])

        return G

    def planning(self, env):

        if len(env.Trees)<=2:
            print('Not enough landmarks')
            print('{} <= 2'.format(len(env.Trees)))

        #recontain to [xi,yi] list
        self.landmarks = [[ti[0],ti[1]] for ti in env.Trees]

        print('I) Make Initial Graph')
        G = self.DelaunayNetwork()  #Delaynay Network Graph
        print('|V|={},|E|={}'.format(G.number_of_nodes(),G.number_of_edges()))
        

        print('II) Modify to Eulerian')
        print('II-i) Extraction odd nodes')
        odds = [v for v in G if G.degree(v)%2==1] #extraction odd nodes
        sg = nx.subgraph(G, odds) #odd nodes subgraph
        print('|Vo|={}'.format(len(odds)))

        print('II-ii) Compute additional edges')
        #make complete graph using odds
        Gc = nx.Graph()
        for v in itertools.combinations(sg.nodes(), 2):
            Gc.add_edge(v[0], v[1])
        #maximal matching
        M = nx.maximal_matching(Gc)
        print('|M|:{}'.format(len(M)))

        Ge = nx.MultiGraph()
        Ge.add_edges_from(G.edges())
        Ge.add_edges_from(M)

        if isShow:
            nx.draw_networkx(G, pos=self.landmarks, ax=self.ax_G, node_color='cyan')
            nx.draw_networkx_nodes(sg, pos=self.landmarks, nodeList=odds, ax=self.ax_G, node_color='red')

            nx.draw_networkx(Ge, pos=self.landmarks, ax=self.ax_Ge, node_color='cyan')
            nx.draw_networkx_nodes(sg,pos=self.landmarks,nodeList=odds,ax=self.ax_Ge,node_color='red')
            nx.draw_networkx_edges(Ge,pos=self.landmarks,edgelist=M,width=3.0,edge_color='red',ax=self.ax_Ge)

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
        
