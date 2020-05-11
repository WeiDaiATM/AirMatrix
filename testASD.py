from Matrix import MatrixBuilder
from PathFinder import AStarDynamic
from SimulationEngine import TrafficGenerator, RandomAircraftCreator
import time
import copy

aircraftList = RandomAircraftCreator.AircraftDataBase()
airmatrix = MatrixBuilder.Matrix((3000, 3000, 2000))
airmatrix.NodeListConstructor()
airmatrix.MatrixConstructor()
traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList, 100)

planner = AStarDynamic.MultiASD(airmatrix, traffic)

time_start = time.time()
planner.MultiSearch()
time_end = time.time()
print('time cost', time_end-time_start, 's')
