
'''
Testing A Star with Obstacles
'''
from Matrix import MatrixBuilder
from PathFinder import AStarwObstacle
from SimulationEngine import TrafficGenerator
from SimulationEngine import RandomAircraftCreator
import time

aircraftList = RandomAircraftCreator.AircraftDataBase()
airmatrix = MatrixBuilder.Matrix((5000, 5000, 2000))
airmatrix.MatrixConstructor()

traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList, 100)
solver = AStarwObstacle.MultiASwO(airmatrix, traffic, 3600)

time_start = time.time()
solver.MultiSearch()
time_end = time.time()
print('time cost', time_end-time_start, 's')

# printing result
for result in solver.planResult:
    duration = result.trajectory[len(result.trajectory)-1][2] - result.trajectory[0][2]
    print(duration)
