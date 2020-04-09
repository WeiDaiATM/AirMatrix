# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 11:43:17 2020

@author: WeiDai

A-Star algorithm with time cost
time is computed from distance and aircraft speed.

"""
# import pandas as np

class Point:
    def __init__(self,x,y,z):
        self.x=x;self.y=y;self.z=z
 
    def __eq__(self, other):
        if self.x==other.x and self.y==other.y and self.z==other.z:
            return True
        return False
    def __str__(self):
        return "x:"+str(self.x)+",y:"+str(self.y)+",z:"+str(self.z)
                       
    
class Node:
    def __init__(self, point, endPoint, indexRange, g=0):
        self.point = point  # coordinate
        self.g = g
        self.h = (abs(endPoint[0] - point[0]) + abs(endPoint[1] - point[1]) + abs(endPoint[2] - point[2]))
        # self.hIndex = (abs(endPoint[0] - point[0]),abs(endPoint[1] - point[1]),abs(endPoint[2] - point[2]))  # manhattan distance
        self.index = int(point[0]+point[1]*indexRange[0]+point[2]*indexRange[0]*indexRange[1])
        self.father = None
        
class Aircraft:
    def __init__(self, speed):
        """
        :param speed: (horizontal, vertical, 45degree, 35degree)
        """
        self.speed = speed
        
        
class AStar:
    # 3-d a star algorithm
    def __init__(self, matrix, startPoint, endPoint, aircraft):
        """
        :param matrix: pre-generated matrix object
        :param startPoint: (x,y,z)
        :param endPoint: (x,y,z)
        :param aircraft: aircraft object
        """
        self.openList = []
        self.closeList = []
        self.matrix = matrix
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.matrix = matrix
        self.aircraft = aircraft
    
    def GetMinNode(self):
        """
        get the node with min F value in openList
        :return: Node
        """
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode
    
    def PointInCloseList(self, point):
        """
        see if node is in open list
        """
        for node in self.closeList:
            if node.point == point:
                return True
        return False
 
    def PointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None
    
    def EndPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None
    
    def Search(self):
        # add startPoint to openList
        startNode = Node(self.startPoint, self.endPoint, self.matrix.indexRange)
        self.openList.append(startNode)
        # search
        while True:
            # find minF
            minF = self.GetMinNode()
            
            # minF to closeList
            self.closeList.append(minF)
            self.openList.remove(minF)
            
            # test the neighbour of minF
            neighbours = self.matrix.network[minF.index]
            for next in neighbours:
                currentPoint = Point(self.matrix.nodeList[next[0]]["index"])
                if self.PointInCloseList(currentPoint):# ignore if in close list
                    continue
                nextNode = Node(currentPoint, self.endPoint, self.matrix.indexRange)
                # nextNode.h = nextNode.hIndex[0]*self.matrix.cellLength + nextNode.hIndex[1]*self.matrix.cellWidth + nextNode.hIndex[2]*self.matrix.cellHeight
                nextNode.g = minF.g + next[1]/self.aircraft.speed[next[2]]

                existNode = self.PointInOpenList(currentPoint) # return if the node exist in open list
                if not existNode:
                    nextNode.father = minF
                    self.openList.append(nextNode)
                    continue
                if nextNode.g < existNode.g: # set g and father if the node exist in open list
                    existNode.g = nextNode.g
                    existNode.father = minF
            lastNode = self.EndPointInCloseList()
            if lastNode:# if endPoint is in close list, return result
                pathList = []
                while True:
                    if lastNode.father:
                        pathList.append(lastNode.point)
                        lastNode = lastNode.father
                    else:
                        return list(reversed(pathList))
            if len(self.openList)==0:
                return None



    