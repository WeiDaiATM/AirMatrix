from PathFinder import AStarwObstacle, AStar

class FourDimensionalAStar(AStar.AStarClassic):
    def __init__(self, matrix, startPoint, endPoint, aircraft, departureTime, dynamicObstacles, odPairs=None):
        super(FourDimensionalAStar, self).__init__(matrix, startPoint, endPoint, aircraft)
        self.departureTime = departureTime
        self.dynamicObstacles = dynamicObstacles
        self.odPairs = odPairs

    def Search(self):
