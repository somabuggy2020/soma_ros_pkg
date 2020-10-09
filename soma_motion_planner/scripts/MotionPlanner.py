#!/usr/bin/env python
import logging as log
from scipy.spatial import KDTree
import math
import matplotlib.pyplot as plt
import networkx as nx
from Environment import Environment
from GraphBasedPathPlanner import GraphBasedPathPlanner
import DubinsPath

log.basicConfig(level=log.DEBUG, format='[%(module)s:%(funcName)s(%(lineno)s)] %(message)s')

LOCAL_PATH_PLANNER = 'DubinsPath'

class MotionPlanner:
    def __init__(self,):
        self.total_length = 0

        self.fig = plt.figure(figsize=(8,8))
        self.fig.canvas.set_window_title('Optimized Trajectory')
        self.ax = plt.subplot(1,1,1)
        pass

    def PoseSampling(self, way_points, nh):
        Q = []
        for k in range(len(way_points)):
            x_k = way_points[k][0]
            y_k = way_points[k][1]
            
            for h in range(nh):
                ang = h/(nh-1)*360
                ang = ang/180.0*math.pi
                ang_k = math.atan2(math.sin(ang), math.cos(ang))
                ang_k = round(ang_k, 2)
                
                q_kh = [x_k, y_k, ang_k] #sampled pose
                Q.append(q_kh)
                
        return Q #sampled pose list

    def planning(self,waypoints,nh,curvature):
        log.info('Create Q as list of states ==========')
        Q = self.PoseSampling(waypoints,nh)
        log.info('|Q|={}'.format(len(Q)))

        log.info('Make pose graph ==========')
        Gq = nx.DiGraph() #pose graph (Directional graph)

        for k in range(len(waypoints)):
            for h in range(nh):
                Gq.add_node(h + k*nh)
        log.debug('|V(Gq)|={}'.format(Gq.number_of_nodes()))

        for k in range(len(waypoints)-1):
            for h in range(nh):
                for i in range(nh):
                    qi = Q[h + k*nh]
                    qj = Q[i + (k+1)*nh]
                    
                    #Dubins Path Algorithm
                    if LOCAL_PATH_PLANNER=='DubinsPath':
                        px,py,pyaw,mode,length = DubinsPath.dubins_path_planning(
                                                    qi[0],qi[1],qi[2],
                                                    qj[0],qj[1],qj[2],
                                                    c=curvature)
                    #set edge weight
                    Gq.add_edge(h+k*nh, i+(k+1)*nh, weight=length)

        log.debug('|V(Gq)|={}, |E(Gq)|={}'.format(Gq.number_of_nodes(), Gq.number_of_edges())) 


        shortest_path = nx.shortest_path(Gq,
                                    source=0,
                                    target=Gq.number_of_nodes()-1,
                                    weight='weight')

        log.info('Trajectory:{}'.format(len(shortest_path)))
        log.debug('->{}'.format(shortest_path))

        traj = []
        yaws = []

        for i in range(len(shortest_path)):
            if i == len(shortest_path)-1:
                break
            qi = Q[shortest_path[i]]
            qj = Q[shortest_path[i+1]]

            #Dubins Path Algorithm
            if LOCAL_PATH_PLANNER=='DubinsPath':
                px,py,pyaw,mode,length = DubinsPath.dubins_path_planning(
                                            qi[0],qi[1],qi[2],
                                            qj[0],qj[1],qj[2],
                                            c=curvature)
        
            traj = traj + [(xi,yi) for xi,yi in zip(px,py)]
            yaws = yaws + pyaw
            self.total_length += length


        self.total_length = round(self.total_length, 2)
        print('Total length:{}'.format(self.total_length))

        self.ax.plot([xi for xi,yi in traj],
                    [yi for xi,yi in traj],
                    '--r')
        plt.show()

        return traj

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
    trajectory = motionPlanner.planning(waypoints,3,1.0)


