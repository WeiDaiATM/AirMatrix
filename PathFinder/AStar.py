# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 11:43:17 2020

@author: WeiDai

A-Star algorithm with time cost
time is computed from distance and aircraft speed.

"""
import numpy as np
from Matrix import MatrixBuilder


# import pandas as np

class Point(object):
    def __init__(self, index):
        self.x = index[0]
        self.y = index[1]
        self.z = index[2]

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z:
            return True
        return False

    def __str__(self):
        return "x:" + str(self.x) + ",y:" + str(self.y) + ",z:" + str(self.z)


class Node(object):
    # def __init__(self, point, endPoint, gainHorizontal=5, gainVertical=10, g=0):
    #     """
    #     :param point: (x,y,z)
    #     :param endPoint: (x,y,z)
    #     :param g: NA
    #     """
    #     self.point = point  # coordinate
    #     self.endPoint = endPoint
    #     self.g = g
    #
    #     # self.h = np.sqrt(gainHorizontal * gainHorizontal * abs(self.endPoint.x - self.point.x) *
    #     #                  abs(self.endPoint.x - self.point.x) + gainHorizontal * gainHorizontal *
    #     #                  abs(self.endPoint.y - self.point.y) * abs(self.endPoint.y - self.point.y) +
    #     #                  gainVertical * gainVertical * abs(self.endPoint.z - self.point.z) *
    #     #                  abs(self.endPoint.z - self.point.z))
    #
    #     self.h = gainHorizontal * abs(self.endPoint.x - self.point.x) + gainHorizontal * \
    #              abs(self.endPoint.y - self.point.y) + gainVertical * abs(self.endPoint.z - self.point.z)
    #     self.father = None

    def __init__(self, point, endPoint, cellLength, cellHeight, maxSpeed, g=0):
        """
        :param point: (x,y,z)
        :param endPoint: (x,y,z)
        :param g: NA
        """
        self.point = point  # coordinate
        self.endPoint = endPoint
        self.g = g

        self.h = np.sqrt((abs(self.endPoint.x - self.point.x) * cellLength) ** 2 +
                         (abs(self.endPoint.y - self.point.y) * cellLength) ** 2 +
                         (abs(self.endPoint.z - self.point.z) * cellHeight) ** 2) / maxSpeed
        self.father = None

    def __eq__(self, newNode):
        if self.point == newNode.point:
            return True
        return False


class Aircraft(object):
    def __init__(self, horizontalMaxSpeed=25, verticalMaxSpeed=5, mass=3):
        """
        :param speed: (horizontal, vertical)
        """
        self.horizontalSpeed = horizontalMaxSpeed
        self.verticalSpeed = verticalMaxSpeed
        self.mass = mass
        self.dragCoef = (self.verticalSpeed * self.mass * 10) / \
                        (self.horizontalSpeed**2 - self.verticalSpeed**2)
        self.pMax = self.verticalSpeed**2 * self.dragCoef * 0.6 # 60% max power
        self.speed = None

    def SetSpeed(self, sinTheta1, sinTheta2):
        """
        used only in A Star path finder initiation
        :param sinTheta1:
        :param sinTheta2:
        :return:
        """
        climbSpeed1 = (-self.mass * 10 * sinTheta1 + np.sqrt(
            self.mass * self.mass * 100 * sinTheta1 * sinTheta1 + 4 * self.dragCoef * self.pMax)) / 2 / self.dragCoef
        climbSpeed2 = (-self.mass * 10 * sinTheta2 + np.sqrt(
            self.mass * self.mass * 100 * sinTheta2 * sinTheta2 + 4 * self.dragCoef * self.pMax)) / 2 / self.dragCoef
        # take 60% of max speed as cruising speed
        self.speed = (self.horizontalSpeed * 0.6, self.verticalSpeed * 0.6, climbSpeed1, climbSpeed2)


class Trajectory(object):
    """
    Turn 3D node list into 4D trajectory
    """

    def __init__(self, matrix, nodeList, aircraft, startTime = 0):
        self.matrix = matrix
        self.nodeList = nodeList
        self.aircraft = aircraft

        self.trajectory = list()
        self.trajectory.append((self.nodeList[0], self.nodeList[0].point.x + self.nodeList[0].point.y
                                                 * matrix.indexRange[0] + self.nodeList[0].point.z * matrix.indexRange[0]
                                                 * matrix.indexRange[1], startTime))

        t = startTime
        # self.nodeList.remove(self.nodeList[0])
        for i in range(len(nodeList)-1):
            node = nodeList[i+1]
            currentIndex = node.point.x + node.point.y * matrix.indexRange[0] + node.point.z * matrix.indexRange[0] * \
                           matrix.indexRange[1]
            links = matrix.FindInNetwork(node.father.point.x + node.father.point.y * matrix.indexRange[0] +
                                    node.father.point.z * matrix.indexRange[0] * matrix.indexRange[1])
            for link in links:
                if link[0] == currentIndex:
                    distance = link[1]
                    speed = aircraft.speed[link[2]]
            t = t + np.ceil(distance / speed)
            self.trajectory.append((node, currentIndex, t))


class AStarClassic(object):
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

        # self.gainHorizontal = matrix.cellLength
        # self.gainVertical = matrix.cellHeight
        self.gainHorizontal = self.matrix.cellWidth / self.aircraft.horizontalSpeed
        self.gainVertical = self.matrix.cellHeight / self.aircraft.verticalSpeed
        self.trajectory = None

    # def __init__(self, matrix, flightPlan):
    #     self.openList = []
    #     self.closeList = []
    #     self.matrix = matrix
    #     self.flightPlan = flightPlan

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
        # startNode = Node(self.startPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
        startNode = Node(self.startPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight, self.aircraft.speed[0])
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
            neighbours = self.matrix.FindInNetwork(int(minF.point.x + minF.point.y *
                                                  self.matrix.indexRange[0] + minF.point.z * self.matrix.indexRange[0] *
                                                  self.matrix.indexRange[1]))
            for nextIndex in neighbours:
                currentPoint = Point((self.matrix.FindInNodelist(nextIndex[0]).x,
                                      self.matrix.FindInNodelist(nextIndex[0]).y, self.matrix.FindInNodelist(nextIndex[0]).z))
                if self.PointInCloseList(currentPoint):  # ignore if in close list
                    continue
                # nextNode = Node(currentPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
                nextNode = Node(currentPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight, self.aircraft.speed[0])
                nextNode.g = minF.g + nextIndex[1] / self.aircraft.speed[nextIndex[2]]

                existNode = self.PointInOpenList(currentPoint)  # return if the node exist in open list
                if not existNode:
                    nextNode.father = minF
                    self.openList.append(nextNode)
                    continue
                if nextNode.g < existNode.g:  # set g and father if the node exist in open list
                    existNode.g = nextNode.g
                    existNode.father = minF
            lastNode = self.EndPointInCloseList()
            if lastNode:  # if endPoint is in close list, return result
                pathList = []
                while True:
                    if lastNode.father:
                        pathList.append(lastNode)
                        lastNode = lastNode.father
                    else:
                        pathList.append(startNode)
                        return list(reversed(pathList))
                        # return Trajectory(self.matrix, list(reversed(pathList)), self.aircraft)
            if len(self.openList) == 0:
                return None


class AStarMultiple(object):
    def __init__(self, matrix, trafficPlan):
        self.matrix = matrix
        self.trafficPlan = trafficPlan
        self.planResult = list()

    def MultiSearch(self):
        for flight in self.trafficPlan.scheduledFlights:
            pathFinder = AStarClassic(self.matrix, flight.startPoint, flight.endPoint, flight.aircraft)
            trajectory = Trajectory(self.matrix, pathFinder.Search(), flight.aircraft, flight.departureTime)
            self.planResult.append(trajectory)
