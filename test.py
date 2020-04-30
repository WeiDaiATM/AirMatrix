


'''
Testing traditional A Star algorithm
'''
from Matrix import MatrixBuilder
from PathFinder import AStar

airmatrix = MatrixBuilder.Matrix((3000, 3000, 2000))
airmatrix.MatrixConstructor()

dji = AStar.Aircraft()
startPoint = AStar.Point((1, 2, 0))
endPoint = AStar.Point((25, 25, 0))

finder = AStar.AStarClassic(airmatrix, startPoint, endPoint, dji)
result = finder.Search()

'''
Testing A Star with Obstacles
'''
from Matrix import MatrixBuilder
from PathFinder import AStar
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

for result in solver.planResult:
    duration = result.trajectory[len(result.trajectory)-1][2] - result.trajectory[0][2]
    print(duration)

for node in result:
    print(node.point.x, node.point.y)

for node in a.trajectory:
    print(node[0].point.x, node.point[0].y, node.point[0].z)