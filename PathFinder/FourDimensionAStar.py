import numpy as np

from PathFinder import AStarwObstacle, AStar

class FourDimensionalAStar(AStar.AStarClassic):
    def __init__(self, matrix, startPoint, endPoint, aircraft, departureTime, dynamicObstacles, odPairs=None):
        super(FourDimensionalAStar, self).__init__(matrix, startPoint, endPoint, aircraft)
        self.departureTime = departureTime
        self.dynamicObstacles = dynamicObstacles
        self.odPairs = odPairs

    def Search(self):
        startNode = AStar.Node(self.startPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight,
                               self.aircraft.speed)
        startNode.timeEntering = self.departureTime
        self.openList.append(startNode)

        while(True):
            # find minF
            minF = self.GetMinNode()

            # minF to closeList
            self.closeList.append(minF)
            self.openList.remove(minF)

            neighbours = self.matrix.FindInNetwork(int(minF.point.x + minF.point.y *
                                                       self.matrix.indexRange[0] + minF.point.z *
                                                       self.matrix.indexRange[0] *
                                                       self.matrix.indexRange[1]))

            for nextIndex in neighbours:
                holding = None
                neighbourPoint = AStar.Point((self.matrix.FindInNodelist(nextIndex[0]).x,
                                              self.matrix.FindInNodelist(nextIndex[0]).y,
                                              self.matrix.FindInNodelist(nextIndex[0]).z))

                # avoid conflicting with the other OD pairs
                if not (neighbourPoint == self.endPoint or neighbourPoint == self.startPoint):
                    if neighbourPoint in self.odPairs:
                        continue
                if self.PointInCloseList(neighbourPoint): # ignore if in close list
                    continue

                timeEntering = int(self.departureTime + minF.g + nextIndex[1] / self.aircraft.speed[nextIndex[2]] / 2)
                neighbourPoint.timeEntering = timeEntering
                timeExiting = int(timeEntering + max((np.sqrt(self.matrix.cellLength ** 2 * 2 +
                                                              self.matrix.cellHeight ** 2)
                                        / self.aircraft.speed[3]),
                                       (np.sqrt(self.matrix.cellLength ** 2 + self.matrix.cellHeight ** 2) /
                                        self.aircraft.speed[3])))

                if not sum(self.dynamicObstacles.dynamicObsList[nextIndex[0], timeEntering:timeExiting]) == 0:# if meet an occupied block
                    for requiredHoldingTime in range(900):
                        if sum(self.dynamicObstacles.dynamicObsList[nextIndex[0],
                                   (timeEntering+requiredHoldingTime):(timeExiting+requiredHoldingTime)]) == 0:# required minimum holding time
                            holdingTime = requiredHoldingTime
                            continue
                    if holdingTime == 899:# max holding time at each node
                        continue

                    timeEnteringMinF = minF.timeEntering
                    timeExitingMinF = timeEntering + holdingTime
                    if not sum(self.dynamicObstacles.dynamicObsList[nextIndex[0], timeEnteringMinF:timeExitingMinF]) == 0:# if holding not available
                        continue

                    # if holding available
                    nextNode = AStar.Node(neighbourPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight, self.aircraft.speed)
                    nextNode.g = minF.g + nextIndex[1]/self.aircraft.speed[nextIndex[2]] + holdingTime
                    nextNode.timeEntering = timeEntering
                    nextNode.afterHolding = True
                    nextNode.holdingTime = holdingTime

                    existNode = self.PointInOpenList(neighbourPoint)
                    if not existNode:
                        nextNode.father = minF
                        self.openList.append(nextNode)
                        continue
                    if nextNode.g < existNode.g:
                        existNode.g = nextNode.g
                        existNode.timeEntering = nextNode.timeEntering
                        existNode.afterHolding = True
                        existNode.holdingTime = nextNode.holdingTime
                        existNode.father = minF

                else:# if not meet an occupied block
                    nextNode = AStar.Node(neighbourPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight,
                                          self.aircraft.speed)
                    nextNode.g = minF.g + nextIndex[1] / self.aircraft.speed[nextIndex[2]]
                    existNode = self.PointInOpenList(neighbourPoint)  # return if the node exist in open list
                    if not existNode:
                        nextNode.father = minF
                        self.openList.append(nextNode)
                        continue
                    if nextNode.g < existNode.g:  # set g and father if the node exist in open list
                        existNode.g = nextNode.g
                        existNode.father = minF
            lastNode = self.EndPointInCloseList()
            if lastNode:
                pathList = []
                while True:
                    if lastNode.father:
                        pathList.append(lastNode)
                        lastNode = lastNode.father
                    else:
                        pathList.append(startNode)
                        # return list(reversed(pathList))
                        return AStar.Trajectory(self.matrix, list(reversed(pathList)), self.aircraft, self.departureTime)
            if len(self.openList) == 0:
                return None

    # def AddDynamicObstacle(self, trajectory):
    #     startT = trajectory.trajectory[0][2]
    #     atT = trajectory.trajectory[0][2]
    #     for i in range(len(trajectory.trajectory) - 1):
    #         self.dynamicObstacles.SetDynamicObstacle(trajectory.trajectory[i][1], startT,
    #                                                  np.ceil((trajectory.trajectory[i + 1][2] + atT) / 2))
    #         startT = int((trajectory.trajectory[i + 1][2] + atT) / 2)
    #         atT = trajectory.trajectory[i + 1][2]
    #     self.dynamicObstacles.SetDynamicObstacle(trajectory.trajectory[len(trajectory.trajectory) - 1][1], startT,
    #                                              trajectory.trajectory[len(trajectory.trajectory) - 1][2])

class Multi4DAstar(AStarwObstacle.MultiASwO):
    def __init__(self, matrix, traffic, duration=3600, dynamicObstacles=None):
        super(Multi4DAstar, self).__init__(matrix, traffic, duration, dynamicObstacles)

    def MultiSearch(self):
        for flight in self.trafficPlan.scheduledFlights:
            pathFinder = FourDimensionalAStar(self.matrix, flight.startPoint, flight.endPoint, flight.aircraft, flight.departureTime,
                              self.dynamicObstacles, self.odPairs)
            plannedTrajectory = pathFinder.Search()
            self.AddDynamicObstacle(plannedTrajectory)
            self.planResult.append(plannedTrajectory)
