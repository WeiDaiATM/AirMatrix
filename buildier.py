# -*- coding: utf-8 -*-
"""
AirMatrix builder

@author: WeiDai
"""
import numpy as np

# Configs

cellOrigin = (0,0,0)

# cell size
cellLength = 100
cellWidth = 100
cellHeight = 50
horizontalDist = np.sqrt(cellLength*cellLength+cellWidth*cellWidth)
verticalDist = np.sqrt(cellLength*cellLength+cellHeight*cellHeight) # assuming width = length
diagonalDist = np.sqrt(cellLength*cellLength+cellWidth*cellWidth+
                       cellHeight*cellHeight)

# size of total area
matrixRange = (10000,10000,500)
indexRange = (int(matrixRange[0]/cellLength+1), int(matrixRange[1]/cellWidth+1), 
              int(matrixRange[2]/cellHeight+1))

# creating matrix
airMatrix = list()

for z in range(indexRange[2]):
    for y in range(indexRange[1]):
        for x in range(indexRange[0]):
            airMatrix.append(dict(index=(x,y,z),coordinate=
                                  (x*cellLength,y*cellWidth,z*cellHeight)))

def GetMatrix(x,y,z):
    index = int(x+y*indexRange[0]+z*indexRange[0]*indexRange[1])
    return airMatrix[index]

# creating links
def Linking(airMatrix):
    network = list()
    for i in range(len(airMatrix)):
        x,y,z = airMatrix[i]["index"]
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
        ################################
        
        # 0 refers to horizontal
        # 1 refers to vertical
        # 2 refers to 45 degree
        # 3 refers to 35 degree
        
        if lx>=0:
            link.append((int(lx+y*indexRange[0]+z*indexRange[0]*indexRange[1]),cellLength, 0))
            if ly>=0:
                link.append((int(lx+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
                if lz>=0:
                    link.append((int(lx+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 3))
                if hz<=indexRange[2]:
                    link.append((int(lx+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 3))
            
            if lz>=0:
                link.append((int(lx+y*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 2))
            if hz<=indexRange[2]:
                link.append((int(lx+y*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 2))
            if hy<=indexRange[1]:
                link.append((int(lx+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
                if hz<=indexRange[2]:
                    link.append((int(lx+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 3))
                if lz>=0:
                    link.append((int(lx+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 3))
        
        if ly>=0:
            link.append((int(x+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),cellWidth, 0))
            if hz<=indexRange[2]:
                link.append((int(x+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 2))
            if lz>=0:
                link.append((int(x+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 2))
        if lz>=0:
            link.append((int(x+y*indexRange[0]+lz*indexRange[0]*indexRange[1]),cellHeight, 1))
        if hz<=indexRange[2]:
            link.append((int(x+y*indexRange[0]+hz*indexRange[0]*indexRange[1]),cellHeight, 1))
            if hy<=indexRange[1]:
                link.append((int(x+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 2))
        if hy<=indexRange[1]:
            link.append((int(x+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),cellWidth, 0))
            if lz>=0:
                link.append((int(x+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 2))
        
        if hx <=indexRange[0]:
            link.append((int(hx+y*indexRange[0]+z*indexRange[0]*indexRange[1]),cellLength, 0))
            if lz>=0:
                link.append((int(hx+y*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 2))
            if hz<=indexRange[2]:
                link.append((int(hx+y*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 2))
            
            if ly>=0:
                link.append((int(hx+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
                if lz>=0:
                    link.append((int(hx+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 3))
                if hz<=indexRange[2]:
                    link.append((int(hx+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 3))
            
            if hy<=indexRange[1]:
                link.append((int(hx+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
                if lz>=0:
                    link.append((int(hx+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 3))
                if hz<=indexRange[2]:
                    link.append((int(hx+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 3))
                    
        if lx>=0:
            link.append((int(lx+y*indexRange[0]+z*indexRange[0]*indexRange[1]),cellLength, 0))
            if ly>=0:
                link.append((int(lx+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
                if lz>=0:
                    link.append((int(lx+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 3))
                if hz<=indexRange[2]:
                    link.append((int(lx+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 3))
            
            if lz>=0:
                link.append((int(lx+y*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 2))
            if hz<=indexRange[2]:
                link.append((int(lx+y*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 2))
            if hy<=indexRange[1]:
                link.append((int(lx+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
                if hz<=indexRange[2]:
                    link.append((int(lx+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 3))
                if lz>=0:
                    link.append((int(lx+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 3))
        
        # if ly>=0:
        #     link.append((int(x+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),cellWidth, 0))
        #     if hz<=indexRange[2]:
        #         link.append((int(x+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 
        #                      cellHeight/np.sqrt(cellLength*cellLength+cellHeight*cellHeight)))
        #     if lz>=0:
        #         link.append((int(x+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 
        #                      cellHeight/np.sqrt(cellLength*cellLength+cellHeight*cellHeight)))
        # if lz>=0:
        #     link.append((int(x+y*indexRange[0]+lz*indexRange[0]*indexRange[1]),cellHeight, 1))
        # if hz<=indexRange[2]:
        #     link.append((int(x+y*indexRange[0]+hz*indexRange[0]*indexRange[1]),cellHeight, 1))
        #     if hy<=indexRange[1]:
        #         link.append((int(x+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 
        #                      cellHeight/np.sqrt(cellLength*cellLength+cellHeight*cellHeight)))
        # if hy<=indexRange[1]:
        #     link.append((int(x+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),cellWidth, 0))
        #     if lz>=0:
        #         link.append((int(x+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 
        #                      cellHeight/np.sqrt(cellLength*cellLength+cellHeight*cellHeight)))
        
        # if hx <=indexRange[0]:
        #     link.append((int(hx+y*indexRange[0]+z*indexRange[0]*indexRange[1]),cellLength, 0))
        #     if lz>=0:
        #         link.append((int(hx+y*indexRange[0]+lz*indexRange[0]*indexRange[1]),verticalDist, 
        #                      cellHeight/np.sqrt(cellLength*cellLength+cellHeight*cellHeight)))
        #     if hz<=indexRange[2]:
        #         link.append((int(hx+y*indexRange[0]+hz*indexRange[0]*indexRange[1]),verticalDist, 
        #                      cellHeight/np.sqrt(cellLength*cellLength+cellHeight*cellHeight)))
            
        #     if ly>=0:
        #         link.append((int(hx+ly*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
        #         if lz>=0:
        #             link.append((int(hx+ly*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 
        #                          cellHeight/np.sqrt(cellLength*cellLength+cellWidth*cellWidth+cellHeight*cellHeight)))
        #         if hz<=indexRange[2]:
        #             link.append((int(hx+ly*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 
        #                          cellHeight/np.sqrt(cellLength*cellLength+cellWidth*cellWidth+cellHeight*cellHeight)))
            
        #     if hy<=indexRange[1]:
        #         link.append((int(hx+hy*indexRange[0]+z*indexRange[0]*indexRange[1]),horizontalDist, 0))
        #         if lz>=0:
        #             link.append((int(hx+hy*indexRange[0]+lz*indexRange[0]*indexRange[1]),diagonalDist, 
        #                          cellHeight/np.sqrt(cellLength*cellLength+cellWidth*cellWidth+cellHeight*cellHeight)))
        #         if hz<=indexRange[2]:
        #             link.append((int(hx+hy*indexRange[0]+hz*indexRange[0]*indexRange[1]),diagonalDist, 
        #                          cellHeight/np.sqrt(cellLength*cellLength+cellWidth*cellWidth+cellHeight*cellHeight)))
        network.append(link)
    return(network)

matrixLinks = Linking(airMatrix)

