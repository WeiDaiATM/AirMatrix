# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 11:43:17 2020

@author: WeiDai

A Star algorithm with time cost
time is computed from distance and aircraft speed.

startPoint: (x,y)
desitnationPoint: (x,y)
network: matrixLink
aircraftspeed: (horizontal speed, vertical speed, speed with 45 degrees angle, speed with 35 degrees angle)
"""
import pandas as np

class Point:
    def __init__(self,x,y,z):
        self.x=x;self.y=y;self.z=z
 
    def __eq__(self, other):
        if self.x==other.x and self.y==other.y and self.z==other.z:
            return True
        return False
    def __str__(self):
        return "x:"+str(self.x)+",y:"+str(self.y)+",z:"+str(self.z)
                       
    
class Node:  # 描述AStar算法中的节点数据
    def __init__(self, point, endPoint, g=0):
        self.point = point  # coordinate
        self.g = g  
        self.h = (abs(endPoint[0] - point[0]) + abs(endPoint[1] - point[1]) + abs(endPoint[2] - point[2]))  # manhattan distance
        self.index = int(point[0]+point[1]*indexRange[0]+point[2]*indexRange[0]*indexRange[1])
        
class Aircraft:
    def __init__(self, ):
        
        
class AStar:
    # 3-d a star algorithm
    def __init__(self, matrix, startPoint, endPoint):
        """
        :param matrix: pre-generated matrix object
        :param startPoint: (x,y,z)
        :param endPoint: (x,y,z)
        """
        self.openList = []
        self.closeList = []
        self.matrix = matrix
        self.startPoint = startPoint
        self.endPoint = endPoint
    
    def getMinNode(self):
        """
        get the node with min F value in openList
        :return: Node
        """
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode
    
    def pointInCloseList(self, point):
        """
        see if node is in open list
        """
        for node in self.closeList:
            if node.point == point:
                return True
        return False
 
    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None
    
    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None
    
    def search():
        # add startPoint to openList
        startNode = Node(self.startPoint, self.endPoint)
        self.openList.append(startNode)
        # search
        while True:
            # find minF
            minF = self.getMinNode()
            
            # minF to closeList
            self.closeList.append(minF)
            self.openList.remove(minF)
            
            # test the neighbour of minF
            



    