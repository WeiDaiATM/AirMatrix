#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  8 10:25:14 2020

@author: daiwei

Matrix Building class

"""
import numpy as np

class Matrix:
    def __init__(self, matrixRange, cellSize=(100,100,50)):
        """
        :param matrixRange: （areaLength, areaWidth, areaHeight）
        :param cellSize: (cellLength, cellWidth, cellHeight)
        """
        self.matrixRange =  matrixRange
        self.cellLength, self.cellWidth, self.cellHeight = cellSize
        self.horizontalDist = np.sqrt(self.cellLength*self.cellLength+self.cellWidth*self.cellWidth)
        self.verticalDist = np.sqrt(self.cellLength*self.cellLength+self.cellHeight*self.cellHeight) # assuming width = length
        self.diagonalDist = np.sqrt(self.cellLength*self.cellLength+self.cellWidth*self.cellWidth+
                       self.cellHeight*self.cellHeight)
        self.indexRange = (int(matrixRange[0]/self.cellLength+1), int(matrixRange[1]/self.cellWidth+1),
              int(matrixRange[2]/self.cellHeight+1))
        
        self.nodeList = list()
        
        indexRange = self.indexRange
        for z in range(indexRange[2]):
            for y in range(indexRange[1]):
                for x in range(indexRange[0]):
                    self.nodeList.append(dict(index=(x,y,z),coordinate=
                                          (x*self.cellLength,y*self.cellWidth,z*self.cellHeight)))
                    
        self.network = list()
        for i in range(len(self.nodeList)):
            x,y,z = self.nodeList[i]["index"]
            lx = x-1
            hx = x+1
            ly = y-1
            hy = y+1
            lz = z-1
            hz = z+1
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
            
            if lx>=0:
                link.append((int(lx+y*indexRange[0]+z*indexRange[0]*indexRange[1]), self.cellLength, 0))
                if ly>=0:
                    link.append((int(lx+ly*indexRange[0]+z*indexRange[0]*indexRange[1]), self.horizontalDist, 0))
                    if lz>=0:
                        link.append((int(lx+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]),  self.diagonalDist, 3))
                    if hz<=indexRange[2]:
                        link.append((int(lx+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                
                if lz>=0:
                    link.append((int(lx+y*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
                if hz<=indexRange[2]:
                    link.append((int(lx+y*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
                if hy<=indexRange[1]:
                    link.append((int(lx+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),self.horizontalDist, 0))
                    if hz<=indexRange[2]:
                        link.append((int(lx+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                    if lz>=0:
                        link.append((int(lx+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
            
            if ly>=0:
                link.append((int(x+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),self.cellWidth, 0))
                if hz<=indexRange[2]:
                    link.append((int(x+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
                if lz>=0:
                    link.append((int(x+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
            if lz>=0:
                link.append((int(x+y*indexRange[0]+lz*indexRange[0]*indexRange[1]),self.cellHeight, 1))
            if hz<=indexRange[2]:
                link.append((int(x+y*indexRange[0]+hz*indexRange[0]*indexRange[1]),self.cellHeight, 1))
                if hy<=indexRange[1]:
                    link.append((int(x+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
            if hy<=indexRange[1]:
                link.append((int(x+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),self.cellWidth, 0))
                if lz>=0:
                    link.append((int(x+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
            
            if hx <=indexRange[0]:
                link.append((int(hx+y*indexRange[0]+z*indexRange[0]*indexRange[1]),self.cellLength, 0))
                if lz>=0:
                    link.append((int(hx+y*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
                if hz<=indexRange[2]:
                    link.append((int(hx+y*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
                
                if ly>=0:
                    link.append((int(hx+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),self.horizontalDist, 0))
                    if lz>=0:
                        link.append((int(hx+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                    if hz<=indexRange[2]:
                        link.append((int(hx+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                
                if hy<=indexRange[1]:
                    link.append((int(hx+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),self.horizontalDist, 0))
                    if lz>=0:
                        link.append((int(hx+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                    if hz<=indexRange[2]:
                        link.append((int(hx+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                        
            if lx>=0:
                link.append((int(lx+y*indexRange[0]+z*indexRange[0]*indexRange[1]),self.cellLength, 0))
                if ly>=0:
                    link.append((int(lx+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),self.horizontalDist, 0))
                    if lz>=0:
                        link.append((int(lx+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                    if hz<=indexRange[2]:
                        link.append((int(lx+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                
                if lz>=0:
                    link.append((int(lx+y*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
                if hz<=indexRange[2]:
                    link.append((int(lx+y*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.verticalDist, 2))
                if hy<=indexRange[1]:
                    link.append((int(lx+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),self.horizontalDist, 0))
                    if hz<=indexRange[2]:
                        link.append((int(lx+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
                    if lz>=0:
                        link.append((int(lx+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]), self.diagonalDist, 3))
            self.network.append(link)
            
