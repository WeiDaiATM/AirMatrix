#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 13:12:14 2020
@author: daiwei
TODO: low efficiency when network is large. Need to lower the computation burden in complex mission (when shortest way is blocked).
"""
import numpy as np

from PathFinder import AStar, DynamicObstacle

class ASwO(AStar.AStarClassic):
    """
    algorithm for 4D trajectory planning using  A Star with obstacles (ASwO).
    used as baseline for trajectory plan efficiency
    """
    def __init__(self, matrix, startPoint, endPoint, aircraft, startTime, dynamicObstacles, odPairs = None, maxDuration=900, endAvoidance = None):
        """

        :param matrix:
        :param startPoint:
        :param endPoint:
        :param aircraft:
        :param startTime:
        :param maxDuration:
        :param dynamicObstacles:
        """
        super(ASwO, self).__init__(matrix, startPoint, endPoint, aircraft)
        self.departureTime = startTime
        self.activeObstacles = dynamicObstacles.dynamicObsList[:, startTime:(startTime+maxDuration)]
        self.startTime = startTime
        self.endAvoidance = endAvoidance
        self.odPairs = odPairs

    # def Search(self):
    #     # plan trajectory with regading obstacles
    #     # add startPoint to openList
    #     startNode = AStar.Node(self.startPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
    #     self.openList.append(startNode)
    #     # search
    #     while True:
    #         # find minF
    #         minF = self.GetMinNode()
    #
    #         # minF to closeList
    #         self.closeList.append(minF)
    #         self.openList.remove(minF)
    #
    #         # test the neighbour of minF
    #         # neighbours = self.matrix.network[minF.index]
    #         neighbours = self.matrix.network[int(minF.point.x + minF.point.y *
    #                                              self.matrix.indexRange[0] + minF.point.z * self.matrix.indexRange[0] *
    #                                              self.matrix.indexRange[1])]
    #         for nextIndex in neighbours:
    #             if not sum(self.activeObstacles[nextIndex[0], :]) == 0:
    #                 continue
    #             currentPoint = AStar.Point(self.matrix.nodeList[nextIndex[0]]["index"])
    #             if self.PointInCloseList(currentPoint):  # ignore if in close list
    #                 continue
    #             nextNode = AStar.Node(currentPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
    #             nextNode.g = minF.g + nextIndex[1] / self.aircraft.speed[nextIndex[2]]
    #
    #             existNode = self.PointInOpenList(currentPoint)  # return if the node exist in open list
    #             if not existNode:
    #                 nextNode.father = minF
    #                 self.openList.append(nextNode)
    #                 continue
    #             if nextNode.g < existNode.g:  # set g and father if the node exist in open list
    #                 existNode.g = nextNode.g
    #                 existNode.father = minF
    #         lastNode = self.EndPointInCloseList()
    #         if lastNode:  # if endPoint is in close list, return result
    #             pathList = []
    #             while True:
    #                 if lastNode.father:
    #                     pathList.append(lastNode.point)
    #                     lastNode = lastNode.father
    #                 else:
    #                     return AStar.Trajectory(self.matrix, list(reversed(pathList)), self.aircraft, self.departureTime)
    #         if len(self.openList) == 0:
    #             return None

    def Search(self):
        # add startPoint to openList
        # startNode = AStar.Node(self.startPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
        startNode = AStar.Node(self.startPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight, self.aircraft.speed[0])

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
                neighbourPoint = AStar.Point((self.matrix.FindInNodelist(nextIndex[0]).x,
                                       self.matrix.FindInNodelist(nextIndex[0]).y,
                                       self.matrix.FindInNodelist(nextIndex[0]).z))
                if not (neighbourPoint == self.endPoint or neighbourPoint == self.startPoint):
                    if neighbourPoint in self.odPairs:
                        continue

                if not sum(self.activeObstacles[nextIndex[0], :]) == 0:
                    continue

                currentPoint = AStar.Point((self.matrix.FindInNodelist(nextIndex[0]).x,
                                      self.matrix.FindInNodelist(nextIndex[0]).y, self.matrix.FindInNodelist(nextIndex[0]).z))
                if self.PointInCloseList(currentPoint):  # ignore if in close list
                    continue
                # nextNode = AStar.Node(currentPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
                nextNode = AStar.Node(currentPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight, self.aircraft.speed[0])

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
                        # return list(reversed(pathList))
                        return AStar.Trajectory(self.matrix, list(reversed(pathList)), self.aircraft, self.startTime)
            if len(self.openList) == 0:
                return "None"

class MultiASwO(object):
    def __init__(self, matrix, traffic, duration, dynamicObstacles = None):
        self.matrix = matrix
        self.odPairs = traffic.odPairs
        self.trafficPlan = traffic.trafficPlan
        if dynamicObstacles:
            self.dynamicObstacles = dynamicObstacles
        else:
            self.dynamicObstacles = DynamicObstacle.DynamicObstacles(self.matrix.nodeList[len(self.matrix.nodeList)-1].index, duration)
        self.planResult = list()

    def AddDynamicObstacle(self, trajectory):
        startT = trajectory.trajectory[0][2]
        atT = 0
        for i in range(len(trajectory.trajectory)-1):
            self.dynamicObstacles.SetDynamicObstacle(trajectory.trajectory[i][1], startT,
                                                     np.ceil((trajectory.trajectory[i+1][2]-atT)/2))
            startT = (trajectory.trajectory[i+1][2]-atT)/2
            atT = trajectory.trajectory[i][2]
        self.dynamicObstacles.SetDynamicObstacle(trajectory.trajectory[len(trajectory.trajectory)-1][1], startT,
                                                 trajectory.trajectory[len(trajectory.trajectory)-1][2])

    def MultiSearch(self):
        i = 1
        for flight in self.trafficPlan.scheduledFlights:
            pathFinder = ASwO(self.matrix, flight.startPoint, flight.endPoint, flight.aircraft, flight.departureTime,
                              self.dynamicObstacles, self.odPairs)
            plannedTrajectory = pathFinder.Search()
            self.AddDynamicObstacle(plannedTrajectory)
            self.planResult.append(plannedTrajectory)
            print(i)
            i = i + 1