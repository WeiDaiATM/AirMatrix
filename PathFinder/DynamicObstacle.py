
import numpy as np
#from Matrix import MatrixBuilder


#class OccupiedCell(MatrixBuilder.Cell):
#    def __init__(self):



class DynamicObstacles(object):
    def __init__(self, maxNoNode, noSec):
        self.dynamicObsList = np.zeros((maxNoNode, noSec), dtype="uint8")

    def SetDynamicObstacle(self, index, startTime, endTime):
        self.dynamicObsArray[index, startTime:endTime] = 1



