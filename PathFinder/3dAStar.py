# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 11:43:17 2020

@author: WeiDai

A-Star algorithm with time cost
time is computed from distance and aircraft speed.

"""
import numpy as np
# import pandas as np
class Point:
    def __init__(self, index):
        self.x = index[0]
        self.y = index[1]
        self.z = index[2]

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        return False

    def __str__(self):
        return "x:"+str(self.x)+",y:"+str(self.y)+",z:"+str(self.z)


class Node:
    def __init__(self, point, endPoint, gainHorizontal = 5, gainVertical = 10, g=0):
        """
        :param point: (x,y,z)
        :param endPoint: (x,y,z)
        :param g: NA
        """
        self.point = point  # coordinate
        self.endPoint = endPoint
        self.g = g
        self.h = gainHorizontal * abs(self.endPoint.x - self.point.x) + gainHorizontal * \
                 abs(self.endPoint.y - self.point.y) + gainVertical * abs(self.endPoint.z - self.point.z)
        self.father = None

class Aircraft:
    def __init__(self, horizontalSpeed, verticalSpeed, mass):
        """
        :param speed: (horizontal, vertical)
        """
        self.horizontalSpeed = horizontalSpeed
        self.verticalSpeed = verticalSpeed
        self.mass = mass
        self.dragCoef = (self.verticalSpeed * self.mass * 10) / \
                    (self.horizontalSpeed * self.horizontalSpeed - self.verticalSpeed * self.verticalSpeed)
        self.pMax = self.verticalSpeed * self.verticalSpeed * self.dragCoef
        self.speed = None

    def SetSpeed(self, sinTheta1, sinTheta2):
        climbSpeed1 = (-self.mass * 10 * sinTheta1 + np.sqrt(
            self.mass * self.mass * 100 * sinTheta1 * sinTheta1 + 4 * self.dragCoef * self.pMax)) / 2 / self.dragCoef
        climbSpeed2 = (-self.mass * 10 * sinTheta2 + np.sqrt(
            self.mass * self.mass * 100 * sinTheta2 * sinTheta2 + 4 * self.dragCoef * self.pMax)) / 2 / self.dragCoef
        self.speed = (self.horizontalSpeed, self.verticalSpeed, climbSpeed1, climbSpeed2)


class AStar:
    # 3-d a star algorithm
    def __init__(self, matrix, startPoint, endPoint, aircraft):
        """
        :param matrix: pre-generated atrix object
        :param startPoint: Point(x,y,z)
        :param endPoint: Point(x,y,z)
        :param aircraft: aircraft object
        """
        self.openList = []
        self.closeList = []
        self.matrix = matrix
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.matrix = matrix
        self.aircraft = aircraft
        self.aircraft.SetSpeed(self.matrix.sinTheta1, self.matrix.sinTheta2)
        self.gainHorizontal = self.matrix.cellWidth / self.aircraft.horizontalSpeed
        self.gainVertical = self.matrix.cellHeight / self.aircraft.verticalSpeed

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
        startNode = Node(self.startPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
        self.openList.append(startNode)
        # search
        while True:
            # find minF
            minF = self.GetMinNode()

            # minF to closeList
            self.closeList.append(minF)
            self.openList.remove(minF)

            # test the neighbour of minF
            # neighbours = self.matrix.network[minF.index]
            neighbours = self.matrix.network[int(minF.point.x + minF.point.y *
                                                 self.matrix.indexRange[0] + minF.point.z * self.matrix.indexRange[0] *
                                                 self.matrix.indexRange[1])]
            for nextIndex in neighbours:
                currentPoint = Point(self.matrix.nodeList[nextIndex[0]]["index"])
                if self.PointInCloseList(currentPoint):# ignore if in close list
                    continue
                nextNode = Node(currentPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
                nextNode.g = minF.g + nextIndex[1]/self.aircraft.speed[nextIndex[2]]

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



