from Matrix import MatrixBuilder
import random
import numpy as np

class BuildingGenerator(object):
    def __init__(self):
        self.matrix = None
        self.staticObsList = list()

    def Generate(self, matrix, noBuilding=50, maxLength=100, minLength=10, maxWidth=100, minWidth=10, maxHeight=500, minHeight=10):
        for i in range(noBuilding):
            length = random.randint(minLength, maxLength)
            width = random.randint(minWidth, maxWidth)
            height = random.randint(minHeight, maxHeight)
            lon = random.randint(np.ceil(length/2), matrix.cellWidth * matrix.indexRange[0])
            lat = random.randint(np.ceil(width/2), matrix.cellLength * matrix.indexRange[1])
            self.AddObstacle(MatrixBuilder.Building(lon, lat, length, width, height))

    def AddObstacle(self, obs):
        self.staticObsList.append(obs)

    def CellInObstacle(self, cell):
        for obs in self.staticObsList:
            # if a cell and a building overlap each other
            if not ((cell.alt - 0.5 * cell.height) >= obs.height or (
                    cell.lon - 0.5 * cell.length) >= (obs.lon + 0.5 * obs.length) or (
                    cell.lon + 0.5 * cell.length) < (obs.lon - 0.5 * obs.length) or (
                    cell.lat - 0.5 * cell.width) >= (obs.lat + 0.5 * obs.width) or (
                    cell.lat + 0.5 * cell.width) < (obs.lat - 0.5 * obs.width)):
                return True
        return False
