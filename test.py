


'''
Testing traditional A Star algorithm
'''
from Matrix import MatrixBuilder
from PathFinder import AStar
from PathFinder import AStarwObstacle
from SimulationEngine import TrafficGenerator
from SimulationEngine import RandomAircraftCreator

airmatrix = MatrixBuilder.Matrix((3000, 3000, 1000))
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

aircraftList = RandomAircraftCreator.AircraftDataBase()
airmatrix = MatrixBuilder.Matrix((3000, 3000, 1000))
airmatrix.MatrixConstructor()

traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList)
solver = AStarwObstacle.MultiASwO(airmatrix, traffic.trafficPlan, 3600)
solver.MultiSearch()
