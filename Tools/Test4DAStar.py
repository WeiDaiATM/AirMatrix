from Matrix import MatrixBuilder, BuildingGenerator
from PathFinder import AStar
from PathFinder import Dijkstra
from PathFinder import FourDimensionAStar
from PathFinder import DynamicObstacle
from SimulationEngine import TrafficGenerator
from SimulationEngine import RandomAircraftCreator
import time

import numpy as np
import pandas as pd

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

aircraftList = RandomAircraftCreator.AircraftDataBase()
airmatrix = MatrixBuilder.Matrix((3000, 3000, 2000))
obstacle = BuildingGenerator.BuildingGenerator()
obstacle.Generate(airmatrix, 100, 100, 20, 100, 20, 500, 80)
airmatrix.NodeListConstructor(obstacle)
airmatrix.MatrixConstructor()
traffic = TrafficGenerator.TrafficGenerator(airmatrix, aircraftList, 300)

dynamicObstacles = DynamicObstacle.DynamicObstacles(airmatrix.nodeList[len(airmatrix.nodeList)-1].index, 3600)

def AddDynamicObstacle(dynamicObstacles, trajectory):
    startT = trajectory.trajectory[0][2]
    atT = trajectory.trajectory[0][2]
    for i in range(len(trajectory.trajectory)-1):
        dynamicObstacles.SetDynamicObstacle(trajectory.trajectory[i][1], startT, np.ceil((trajectory.trajectory[i+1][2] + atT) / 2))
        startT = int((trajectory.trajectory[i+1][2] + atT) / 2)
        atT = trajectory.trajectory[i + 1][2]
    dynamicObstacles.SetDynamicObstacle(trajectory.trajectory[len(trajectory.trajectory)-1][1], startT, trajectory.trajectory[len(trajectory.trajectory)-1][2])
    return dynamicObstacles


result = pd.DataFrame()
time_accu_d = 0
time_accu_a = 0
time_accu_4d = 0
delay_accu_4d = 0

result_d = []
result_a = []
result_4d = []

for flight in traffic.trafficPlan.scheduledFlights:
    time_start = time.time()
    finder_d = Dijkstra.Dij(airmatrix, flight.startPoint, flight.endPoint, flight.aircraft)
    trajectory_d = AStar.Trajectory(airmatrix, finder_d.Search(), flight.aircraft, flight.departureTime)
    time_end = time.time()
    time_d = time_end - time_start
    time_accu_d += time_d
    result_d.append(trajectory_d)

    aobt_d = trajectory_d.trajectory[0][2]
    aibt_d = trajectory_d.trajectory[len(trajectory_d.trajectory) - 1][2]
    cost_d = aibt_d - aobt_d

    time_start = time.time()
    finder_a = AStar.AStarClassic(airmatrix, flight.startPoint, flight.endPoint, flight.aircraft)
    trajectory_a = AStar.Trajectory(airmatrix, finder_a.Search(), flight.aircraft, flight.departureTime)
    time_end = time.time()
    time_a = time_end - time_start
    time_accu_a += time_a
    result_a.append(trajectory_a)

    aobt_a = trajectory_a.trajectory[0][2]
    aibt_a = trajectory_a.trajectory[len(trajectory_a.trajectory) - 1][2]
    cost_a = aibt_a - aobt_a

    time_start = time.time()
    finder_4d = FourDimensionAStar.FourDimensionalAStar(airmatrix, flight.startPoint, flight.endPoint, flight.aircraft,
                                                        flight.departureTime, dynamicObstacles, traffic.odPairs)
    trajectory_4d = finder_4d.Search()

    if (not trajectory_4d):
        break
    result_4d.append(trajectory_4d)

    time_end = time.time()
    time_4d = time_end - time_start
    time_accu_4d += time_4d
    dynamicObstacles = AddDynamicObstacle(dynamicObstacles, trajectory_4d)

    aobt_4d = trajectory_4d.trajectory[0][2]
    aibt_4d = trajectory_4d.trajectory[len(trajectory_4d.trajectory) - 1][2]
    cost_4d = aibt_4d - aobt_4d
    delay_4d = cost_4d - cost_a
    delay_accu_4d += delay_4d

    new_data = {'Aircraft': flight.aircraft.name, 'Dijkstra Cost': cost_d, 'Dijkstra Computation Time': time_d,
                'Dijkstra Accumulated Computation Time': time_accu_d, 'A-Star Cost': cost_a,
                'A-Star Computation Time': time_a, 'A-Star Accumulated Computation Time': time_accu_a,
                'CFA* Cost': cost_4d, 'CFA* Computation Time': time_4d,
                'CFA* Accumulated Computation Time': time_accu_4d,
                'CFA* Delay': delay_4d,
                'CFA* Accumulated Delay': delay_accu_4d}

    result = result.append(new_data, ignore_index=True)