from Matrix import MatrixBuilder
from PathFinder import AStar
from SimulationEngine import TrafficGenerator
from SimulationEngine import RandomAircraftCreator
import time

aircraftList = RandomAircraftCreator.AircraftDataBase()
airmatrix = MatrixBuilder.Matrix((3000, 3000, 2000))
airmatrix.MatrixConstructor()
traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList, 100)
finder = AStar.AStarMultiple(airmatrix, traffic.trafficPlan)
time_start = time.time()
finder.MultiSearch()
time_end = time.time()
print('time cost', time_end-time_start, 's')