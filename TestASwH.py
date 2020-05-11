from Matrix import MatrixBuilder
from PathFinder import AStarWithHolding
from SimulationEngine import TrafficGenerator
from SimulationEngine import RandomAircraftCreator
import time
import copy

aircraftList = RandomAircraftCreator.AircraftDataBase()
airmatrix = MatrixBuilder.Matrix((3000, 3000, 2000))
airmatrix.MatrixConstructor()
traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList, 100)

planner = AStarWithHolding.AStarWithHolding(airmatrix, traffic)

time_start = time.time()
planner.InitialPlan()
time_end = time.time()
print('time cost', time_end-time_start, 's')
backup = copy.deepcopy(planner.initPlanner.planResult)

time_start = time.time()
planner.ConflictSolver()
time_end = time.time()
print('time cost', time_end-time_start, 's')
