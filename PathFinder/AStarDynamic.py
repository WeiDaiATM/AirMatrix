"""
@author: daiwei
"""
import numpy as np
from PathFinder import AStarwObstacle, AStar

class ASD(AStar.AStarClassic):
    def __init__(self, matrix, startPoint, endPoint, aircraft, departureTime, dynamicObstacles, odPairs=None):
        super(ASD, self).__init__(matrix, startPoint, endPoint, aircraft)
        self.departureTime = departureTime
        self.dynamicObstacles = dynamicObstacles
        self.odPairs = odPairs

    def Search(self):
        startNode = AStar.Node(self.startPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight, self.aircraft.speed)
        self.openList.append(startNode)

        while True:
            # find minF
            minF = self.GetMinNode()

            # minF to closeList
            self.closeList.append(minF)
            self.openList.remove(minF)

            neighbours = self.matrix.FindInNetwork(int(minF.point.x + minF.point.y *
                                                  self.matrix.indexRange[0] + minF.point.z * self.matrix.indexRange[0] *
                                                  self.matrix.indexRange[1]))
            for nextIndex in neighbours:
                neighbourPoint = AStar.Point((self.matrix.FindInNodelist(nextIndex[0]).x,
                                       self.matrix.FindInNodelist(nextIndex[0]).y,
                                       self.matrix.FindInNodelist(nextIndex[0]).z))

                # avoid conflicting with the other OD pairs
                if not (neighbourPoint == self.endPoint or neighbourPoint == self.startPoint):
                    if neighbourPoint in self.odPairs:
                        continue

                timeEntering = int(self.departureTime + minF.g + nextIndex[1] / self.aircraft.speed[nextIndex[2]] / 2)
                timeExiting = int(timeEntering + max((np.sqrt(self.matrix.cellLength ** 2 * 2 + self.matrix.cellHeight ** 2)
                                                  / self.aircraft.speed[3]),
                                                 (np.sqrt(self.matrix.cellLength ** 2 + self.matrix.cellHeight ** 2) /
                                                 self.aircraft.speed[3])))
                if not sum(self.dynamicObstacles.dynamicObsList[nextIndex[0], timeEntering:timeExiting]) == 0:
                    continue
                currentPoint = AStar.Point((self.matrix.FindInNodelist(nextIndex[0]).x,
                                            self.matrix.FindInNodelist(nextIndex[0]).y,
                                            self.matrix.FindInNodelist(nextIndex[0]).z))
                if self.PointInCloseList(currentPoint):  # ignore if in close list
                    continue

                nextNode = AStar.Node(currentPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight, self.aircraft.speed)

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
                        return AStar.Trajectory(self.matrix, list(reversed(pathList)), self.aircraft, self.departureTime)
            if len(self.openList) == 0:
                return "None"

class MultiASD(AStarwObstacle.MultiASwO):
    def __init__(self, matrix, traffic, duration=3600, dynamicObstacles=None):
        super(MultiASD, self).__init__(matrix, traffic, duration, dynamicObstacles)

    def MultiSearch(self):
        i = 1
        for flight in self.trafficPlan.scheduledFlights:
            pathFinder = ASD(self.matrix, flight.startPoint, flight.endPoint, flight.aircraft, flight.departureTime,
                              self.dynamicObstacles, self.odPairs)
            plannedTrajectory = pathFinder.Search()
            self.AddDynamicObstacle(plannedTrajectory)
            self.planResult.append(plannedTrajectory)
            #print(i)
            i = i+1
