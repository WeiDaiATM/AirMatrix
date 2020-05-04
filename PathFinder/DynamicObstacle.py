
import numpy as np

class DynamicObstacles(object):
    def __init__(self, maxNoNode, noSec):
        self.dynamicObsList = np.zeros((maxNoNode, noSec), dtype="uint8")

    def SetDynamicObstacle(self, index, startTime, endTime):
        self.dynamicObsList[index, int(startTime):int(endTime)] = self.dynamicObsList[index, int(startTime):int(endTime)] + 1



