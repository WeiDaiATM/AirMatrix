from Matrix import MatrixBuilder
from PathFinder import AStar
from PathFinder import Dijkstra
from SimulationEngine import TrafficGenerator
from SimulationEngine import RandomAircraftCreator
from Matrix import BuildingGenerator
import time

# no obstacle case
aircraftList = RandomAircraftCreator.AircraftDataBase()
airmatrix = MatrixBuilder.Matrix((3000, 3000, 2000), 100, 50)
airmatrix.NodeListConstructor()
airmatrix.MatrixConstructor()
traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList, 100)

# Dijkstra Algorithm
finder = Dijkstra.MultiDij(airmatrix, traffic.trafficPlan)
time_start = time.time()
finder.MultiSearch()
dijkstra_result = finder.planResult
time_end = time.time()
print('time cost', time_end-time_start, 's')

# AStar Algorithm
finder = AStar.AStarMultiple(airmatrix, traffic.trafficPlan)
time_start = time.time()
finder.MultiSearch()
astar_result = finder.planResult
time_end = time.time()
print('time cost', time_end-time_start, 's')

for i in range(100):
    print("\nTrajectory "+str(i+1)+":")
    dij_start_time = dijkstra_result[i].trajectory[0][2]
    dij_length = dijkstra_result[i].trajectory[0]
    dij_end_time = dijkstra_result[i].trajectory[len(dijkstra_result[i].trajectory)-1][2]
    dij_cost = dij_end_time - dij_start_time
    print("Dijstra cost: "+str(dij_cost)+" seconds")

    astar_start_time = astar_result[i].trajectory[0][2]
    astar_length = astar_result[i].trajectory[0]
    astar_end_time = astar_result[i].trajectory[len(astar_result[i].trajectory)-1][2]
    astar_cost = astar_end_time - astar_start_time
    print("Dijstra cost: "+str(astar_cost)+" seconds")
    print("Difference: "+str(dij_cost-astar_cost))

# obstacle case
airmatrix = MatrixBuilder.Matrix((3000, 3000, 1500), 100, 50)
obstacle = BuildingGenerator.BuildingGenerator()
obstacle.Generate(airmatrix, 100, 100, 10, 100, 10, 500, 10)
airmatrix.NodeListConstructor(obstacle)
airmatrix.MatrixConstructor()
traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList, 100)

# Dijkstra Algorithm
finder = Dijkstra.MultiDij(airmatrix, traffic.trafficPlan)
time_start = time.time()
finder.MultiSearch()
dijkstra_result = finder.planResult
time_end = time.time()
print('time cost', time_end-time_start, 's')

# AStar Algorithm
finder = AStar.AStarMultiple(airmatrix, traffic.trafficPlan)
time_start = time.time()
finder.MultiSearch()
astar_result = finder.planResult
time_end = time.time()
print('time cost', time_end-time_start, 's')
