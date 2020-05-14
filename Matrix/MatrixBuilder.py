#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  8 10:25:14 2020

@author: daiwei

Matrix Building class
"""
import numpy as np

class Building(object):
    def __init__(self, lon, lat, length, width, height):
        """
        :param alt:
        :param lon:
        :param length: x direction (longitude)
        :param width: y direction (latitude)
        :param height:
        """
        self.lon = lon
        self.lat = lat
        self.length = length
        self.width = width
        self.length = length
        self.height = height


# class StaticObstacles(object):
#     def __init__(self):
#         self.staticObsList = list()
#
#     def AddObstacle(self, obs):
#         self.staticObsList.append(obs)
#
#     def CellInObstacle(self, cell):
#         for obs in self.staticObsList:
#             # if a cell and a building overlap each other
#             if (cell.alt - 0.5 * cell.height) >= obs.height or (
#                     cell.lon - 0.5 * cell.length) >= (obs.lon + 0.5 * obs.length) or (
#                     cell.lon + 0.5 * cell.length) < (obs.lon - 0.5 * obs.length) or (
#                     cell.lat - 0.5 * cell.width) >= (obs.lat + 0.5 * obs.width) or (
#                     cell.lat + 0.5 * cell.width) < (obs.lat - 0.5 * obs.width):
#                 return False
#             return True


class Cell(object):
    """
    cell class used in matrix building
    """
    def __init__(self, x, y, z, cellLength, cellHeight):
        self.x = x
        self.y = y
        self.z = z
        self.index = None
        self.length = cellLength
        self.width = cellLength
        self.height = cellHeight

        self.lon = self.x * self.length + 0.5 * self.length
        self.lat = self.y * self.width + 0.5 * self.width
        self.alt = self.z * self.height + 0.5 * self.height

    def SetIndex(self, index):
        self.index = index


class Matrix(object):
    def __init__(self, matrixRange, cellLength=100, cellHeight=100):
        """
        :param matrixRange: (x,y,z)
        :param cellLength:
        :param cellHeight:
        :param obstacles: False to disable static obstacles
        """
        if not(((matrixRange[0] % cellLength) == 0) and ((matrixRange[1] % cellLength) ==0) and
               ((matrixRange[2] % cellHeight) == 0)):
            print("matrix range and size doesn't fit, please re-do it")

        self.matrixRange = matrixRange
        self.cellLength = cellLength
        self.cellWidth = cellLength
        self.cellHeight = cellHeight
        self.horizontalDist = np.sqrt(self.cellLength * self.cellLength + self.cellWidth * self.cellWidth)
        self.verticalDist = np.sqrt(self.cellLength * self.cellLength + self.cellHeight *
                                    self.cellHeight)  # assuming width = length
        self.diagonalDist = np.sqrt(self.cellLength * self.cellLength + self.cellWidth * self.cellWidth +
                                    self.cellHeight * self.cellHeight)
        self.indexRange = (int(matrixRange[0] / self.cellLength), int(matrixRange[1] / self.cellWidth),
                           int(matrixRange[2] / self.cellHeight))

        self.nodeList = list()

        self.sinTheta1 = self.cellHeight / self.verticalDist
        self.sinTheta2 = self.cellHeight / self.diagonalDist

        self.staticObstacles = None
        self.network = None

    def NodeListConstructor(self, obstacles=None):
        self.staticObstacles = obstacles
        indexRange = self.indexRange
        for z in range(indexRange[2]):
            for y in range(indexRange[1]):
                for x in range(indexRange[0]):
                    newCell = Cell(x, y, z, self.cellLength, self.cellHeight)
                    newCell.SetIndex(x + indexRange[0] * y + indexRange[0] * indexRange[1] * z)
                    if obstacles:
                        if not obstacles.CellInObstacle(newCell):
                            self.nodeList.append(newCell)
                    else:
                        self.nodeList.append(newCell)

        self.indexAvailable = np.zeros(indexRange[0] * indexRange[1] * indexRange[2])
        for node in self.nodeList:
            self.indexAvailable[node.index] = 1
        self.indexAvailable = self.indexAvailable == 1

        falses = np.zeros(indexRange[0] * indexRange[1] * 2) # too stupid, need to fix later!!!
        falses = falses == 1
        self.indexAvailable = np.append(self.indexAvailable, falses)

    def MatrixConstructor(self):
        """
        build matrix network(nodes and links) for airmatrix
        :return:
        """
        self.network = list()
        for node in self.nodeList:
            x, y, z = node.x, node.y, node.z
            lx = x - 1
            hx = x + 1
            ly = y - 1
            hy = y + 1
            lz = z - 1
            hz = z + 1
            link = list()

            ################################
            # hovering, currently disabled
            # link.append(i,0)
            #
            # obstacles are not considered
            ################################

            # 0 refers to horizontal
            # 1 refers to vertical
            # 2 refers to 45 degree
            # 3 refers to 35 degree
            indexRange = self.indexRange
            if lx >= 0:
                index = int(lx + y * indexRange[0] + z * indexRange[0] * indexRange[1])
                if self.indexAvailable[index]:
                    link.append((index, self.cellLength, 0))

                if ly >= 0:
                    index = int(lx + ly * indexRange[0] + z * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.horizontalDist, 0))

                    if lz >= 0:
                        index = int(lx + ly * indexRange[0] + lz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                    if hz < indexRange[2]:
                        index = int(lx + ly * indexRange[0] + hz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                if lz >= 0:
                    index = int(lx + y * indexRange[0] + lz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

                if hz < indexRange[2]:
                    index = int(lx + y * indexRange[0] + hz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

                if hy < indexRange[1]:
                    index = int(lx + hy * indexRange[0] + z * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.horizontalDist, 0))

                    if hz < indexRange[2]:
                        index = int(lx + hy * indexRange[0] + hz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                    if lz >= 0:
                        index = int(lx + hy * indexRange[0] + lz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

            if ly >= 0:
                index = int(x + ly * indexRange[0] + z * indexRange[0] * indexRange[1])
                if self.indexAvailable[index]:
                    link.append((index, self.cellWidth, 0))

                if hz < indexRange[2]:
                    index = int(x + ly * indexRange[0] + hz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

                if lz >= 0:
                    index = int(x + ly * indexRange[0] + lz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

            if lz >= 0:
                index = int(x + y * indexRange[0] + lz * indexRange[0] * indexRange[1])
                if self.indexAvailable[index]:
                    link.append((index, self.cellHeight, 1))

            if hz < indexRange[2]:
                index = int(x + y * indexRange[0] + hz * indexRange[0] * indexRange[1])
                if self.indexAvailable[index]:
                    link.append((index, self.cellHeight, 1))

                if hy < indexRange[1]:
                    index = int(x + hy * indexRange[0] + hz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

            if hy < indexRange[1]:
                index = int(x + hy * indexRange[0] + z * indexRange[0] * indexRange[1])
                if self.indexAvailable[index]:
                    link.append((index, self.cellWidth, 0))

                if lz >= 0:
                    index = int(x + hy * indexRange[0] + lz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

            if hx < indexRange[0]:
                index = int(hx + y * indexRange[0] + z * indexRange[0] * indexRange[1])
                if self.indexAvailable[index]:
                    link.append((index, self.cellLength, 0))

                if lz >= 0:
                    index = int(hx + y * indexRange[0] + lz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

                if hz < indexRange[2]:
                    index = int(hx + y * indexRange[0] + hz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

                if ly >= 0:
                    index = int(hx + ly * indexRange[0] + z * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.horizontalDist, 0))

                    if lz >= 0:
                        index = int(hx + ly * indexRange[0] + lz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                    if hz < indexRange[2]:
                        index = int(hx + ly * indexRange[0] + hz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                if hy < indexRange[1]:
                    index = int(hx + hy * indexRange[0] + z * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.horizontalDist, 0))

                    if lz >= 0:
                        index = int(hx + hy * indexRange[0] + lz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                    if hz < indexRange[2]:
                        index = int(hx + hy * indexRange[0] + hz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

            if lx >= 0:
                index = int(lx + y * indexRange[0] + z * indexRange[0] * indexRange[1])
                if self.indexAvailable[index]:
                    link.append((index, self.cellLength, 0))

                if ly >= 0:
                    index = int(lx + ly * indexRange[0] + z * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.horizontalDist, 0))

                    if lz >= 0:
                        index = int(lx + ly * indexRange[0] + lz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                    if hz < indexRange[2]:
                        index = int(lx + ly * indexRange[0] + hz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                if lz >= 0:
                    index = int(lx + y * indexRange[0] + lz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

                if hz < indexRange[2]:
                    index = int(lx + y * indexRange[0] + hz * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.verticalDist, 2))

                if hy < indexRange[1]:
                    index = int(lx + hy * indexRange[0] + z * indexRange[0] * indexRange[1])
                    if self.indexAvailable[index]:
                        link.append((index, self.horizontalDist, 0))

                    if hz < indexRange[2]:
                        index = int(lx + hy * indexRange[0] + hz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))

                    if lz >= 0:
                        index = int(lx + hy * indexRange[0] + lz * indexRange[0] * indexRange[1])
                        if self.indexAvailable[index]:
                            link.append((index, self.diagonalDist, 3))
            self.network.append(link)

    def FindInNetwork(self, index):
        """
        :param index:
        :return: cell referring to the index
        """
        maxIndex = len(self.nodeList)-1
        if self.nodeList[maxIndex].index == index:
            return self.network[maxIndex]
        minIndex = 0
        if self.nodeList[0].index == index:
            return self.network[0]
        i = int(maxIndex/2)
        while True:
            if self.nodeList[i].index == index:
                return self.network[i]
            if self.nodeList[i].index < index:
                minIndex = i
                i = int((maxIndex+i)/2)
            else:
                maxIndex = i
                i = int((minIndex + i)/2)

    def FindInNodelist(self, index):
        """

        :param index:
        :return: neighbours (as a list) referring to the index
        """
        maxIndex = len(self.nodeList) - 1
        if self.nodeList[maxIndex].index == index:
            return self.nodeList[maxIndex]
        minIndex = 0
        if self.nodeList[0].index == index:
            return self.nodeList[0]
        i = int(maxIndex / 2)
        while True:
            if self.nodeList[i].index == index:
                return self.nodeList[i]
            if self.nodeList[i].index < index:
                minIndex = i
                i = int((maxIndex + i) / 2)
            else:
                maxIndex = i
                i = int((minIndex + i) / 2)