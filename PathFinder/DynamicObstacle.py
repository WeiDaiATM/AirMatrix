
import numpy as np
#from Matrix import MatrixBuilder


#class OccupiedCell(MatrixBuilder.Cell):
#    def __init__(self):



class DynamicObstacles(object):
    def __init__(self, noNode, noSec):
        self.dynamicObsList = np.zeros((noNode, noSec), dtype="uint8")

    def SetDynamicObstacle(self, index, startTime, endTime):
        self.dynamicObsList[index, startTime:endTime] = 1



