#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 13:12:14 2020
@author: daiwei
"""


from PathFinder import AStar

class AStarAvoidingObstacles(AStar):
    def __init__(self):
        super(AStarAvoidingObstacles, self).__init__()


        # read obstacles and select obstacles of interest

        # plan trajectory with regading obstacles

