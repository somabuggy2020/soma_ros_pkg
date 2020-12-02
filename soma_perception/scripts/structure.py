#!/usr/bin/env python

import numpy as np

class Point(object):
  def __init__(self, x_init, y_init, pid):
    self.x = x_init
    self.y = y_init
    self.pid = pid

class pSide(object):
  def __init__(self, aPoint, adist):
    self.p = aPoint
    self.dist = adist

class Triangle(object):
  def __init__(self, aPoint):
    self.a, self.b, self.c = self.makeTriangle(aPoint[0], aPoint[1], aPoint[2])
    self.pIDlist = self.matchIDaxis(self.a, self.b, self.c)
    self.dist, self.dir, self.Orientdir = self.calcdetail(self.a.p, self.b.p, self.c.p)
    
  def matchIDaxis(self, a, b, c):
    tmplist = []
    tmplist.append(Point(a.p.x,a.p.y,'a'))
    tmplist.append(Point(b.p.x,b.p.y,'b'))
    tmplist.append(Point(c.p.x,c.p.y,'c'))

    tmplist.sort(key = lambda x : x.y)
    return tmplist
    
  def calcdetail(self, a, b, c):
    cent_x = (a.x+b.x+c.x) / 3.0
    cent_y = (a.y+b.y+c.y) / 3.0

    dist = np.linalg.norm(np.array([cent_x,cent_y]))

    vec_cs = np.array([cent_x,cent_y])
    vec_y = np.array([5,0])
    inner = np.inner(vec_cs, vec_y)
    n = np.linalg.norm(vec_cs) * np.linalg.norm(vec_y)
    rad = np.arccos(inner/n)
    if cent_y < 0:
      rad = np.pi + rad
        
    vec_csa = np.array([(a.x-cent_x), (a.y-cent_y)])
    inner = np.inner(vec_csa, vec_y)
    n = np.linalg.norm(vec_csa) * np.linalg.norm(vec_y)
    radOri = np.arccos(inner/n)
    if (a.y - cent_y) < 0:
      radOri = np.pi + radOri

    return dist, rad, radOri

  def calcheight(self, a, b, c):
    vec_cb = np.array([(b.x-c.x),(b.y-c.y)])
    vec_ca = np.array([(a.x-c.x),(a.y-c.y)]) 

    inner = np.inner(vec_cb, vec_ca) 
    n = np.linalg.norm(vec_cb) * np.linalg.norm(vec_ca)
    rad = np.arccos(inner/n)
    acdist = np.linalg.norm(vec_ca)
    height = abs(acdist * np.sin(rad))

    return height


  def makeTriangle(self, a, b, c):
    abD = np.sqrt((a.x - b.x) **2 + (a.y - b.y) **2)
    bcD = np.sqrt((b.x - c.x) **2 + (b.y - c.y) **2)
    caD = np.sqrt((c.x - a.x) **2 + (c.y - a.y) **2)
    s1 = pSide(a,bcD)
    s2 = pSide(b,caD)
    s3 = pSide(c,abD)

    if(s1.dist > s2.dist):
      if(s1.dist < s3.dist):
        s1, s3 = s3, s1
    elif(s2.dist > s3.dist):
      s1, s2 = s2, s1
    else:
      s1, s3 = s3, s1

    vec_ab = np.array([(s2.p.x - s1.p.x), (s2.p.y - s1.p.y)])
    vec_ac = np.array([(s3.p.x - s1.p.x), (s3.p.y - s1.p.y)])
    cross = np.cross(vec_ab, vec_ac)
    if(cross > 0):
      s2, s3 = s3, s2

    return s1, s2, s3