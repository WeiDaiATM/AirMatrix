from PathFinder import AStar

class Dij(AStar.AStarClassic):
    def __init__(self, matrix, startPoint, endPoint, aircraft, odPairs=None):
        super(Dij, self).__init__(matrix, startPoint, endPoint, aircraft, odPairs)

    def Search(self):
        # add startPoint to openList
        startNode = AStar.Node(self.startPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight,
                         self.aircraft.speed)
        startNode.h = 0
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
                                                       self.matrix.indexRange[0] + minF.point.z *
                                                       self.matrix.indexRange[0] *
                                                       self.matrix.indexRange[1]))
            for nextIndex in neighbours:
                currentPoint = AStar.Point((self.matrix.FindInNodelist(nextIndex[0]).x,
                                      self.matrix.FindInNodelist(nextIndex[0]).y,
                                      self.matrix.FindInNodelist(nextIndex[0]).z))
                # if not (currentPoint == self.endPoint or currentPoint == self.startPoint):
                #     if currentPoint in self.odPairs:
                #         continue
                if self.PointInCloseList(currentPoint):  # ignore if in close list
                    continue
                # nextNode = Node(currentPoint, self.endPoint, self.gainHorizontal, self.gainVertical)
                nextNode = AStar.Node(currentPoint, self.endPoint, self.matrix.cellLength, self.matrix.cellHeight,
                                self.aircraft.speed)
                nextNode.h = 0
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

class MultiDij(AStar.AStarMultiple):
    def __init__(self, matrix, trafficPlan):
        super(MultiDij, self).__init__(matrix, trafficPlan)

    def MultiSearch(self):
        # ct = 1
        for flight in self.trafficPlan.scheduledFlights:
            pathFinder = Dij(self.matrix, flight.startPoint, flight.endPoint, flight.aircraft)
            trajectory = AStar.Trajectory(self.matrix, pathFinder.Search(), flight.aircraft, flight.departureTime)
            self.planResult.append(trajectory)