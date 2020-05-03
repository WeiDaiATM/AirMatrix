#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 13:12:14 2020
@author: daiwei
"""
import numpy as np
from PathFinder import AStar
from PathFinder import AStarwObstacle
from PathFinder import DynamicObstacle

class AStarWithHolding(AStarwObstacle.MultiASwO):
    def __init__(self, matrix, traffic, duration=3600, dynamicObstacles=None):
        super(AStarWithHolding, self).__init__(matrix, traffic, duration, dynamicObstacles)
        # plan trajectory
        self.initPlanner = AStar.AStarMultiple(self.matrix, self.traffic.trafficPlan)
        self.initPlanner.MultiSearch()
        self.planResult = list()
        self.duration = duration

    def conflictSolver(self):
        # solve conflict with holding
        for flightTrajectory in self.initPlanner.planResult:
            newTrajectory = list()

            holdTime = 0
            maxHolding = self.duration - flightTrajectory.trajectory[len(flightTrajectory.trajectory)-1][2]

            startLeg = flightTrajectory.trajectory[0]
            sTime = startLeg[2]
            eTime = np.ceil((startLeg[2] + flightTrajectory.trajectory[1][2]) / 2)

            while(holdTime < maxHolding):
                if sum(self.dynamicObstacle.dynamicObsList[startLeg[1]][sTime + holdTime, eTime + holdTime]) == 0:
                    break
                holdTime = holdTime+1
            newTrajectory.append((startLeg[0], startLeg[1], startLeg[2]+holdTime))

            for i in range(len(flightTrajectory.trajectory)-1):
                nextLeg = flightTrajectory.trajectory[i+1]

                sTime = int((flightTrajectory.trajectory[i][2] + nextLeg[2]) / 2)
                if i < len(flightTrajectory.trajectory)-1:
                    eTime = np.ceil((flightTrajectory.trajectory[i+2][2] + flightTrajectory.trajectory[i+1][i]) / 2)
                else:
                    eTime = flightTrajectory.trajectory[i+1][i]

                while(holdTime < maxHolding):
                    if sum(self.dynamicObstacle.dynamicObsList[nextLeg[1]][sTime+holdTime, eTime+holdTime]) == 0:
                        break
                    holdTime = holdTime + 1

                newTrajectory.append((flightTrajectory.trajectory[i][0], flightTrajectory.trajectory[i][1],
                                flightTrajectory.trajectory[i[2+holdTime]]))
                newTrajectory.append((nextLeg[0], nextLeg[1], nextLeg[2]+holdTime))
