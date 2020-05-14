import networkx as nx
from Matrix import MatrixBuilder, BuildingGenerator
import time

airmatrix = MatrixBuilder.Matrix((1000, 1000, 500), 100, 50)

obstacle = BuildingGenerator.BuildingGenerator()
obstacle.Generate(airmatrix, 15, 100, 10, 100, 10, 100, 10)
airmatrix.NodeListConstructor(obstacle)
airmatrix.MatrixConstructor()

graph = nx.Graph()
graph.add_nodes_from(airmatrix.nodeList)

edges = list()

for node in airmatrix.nodeList:
    index = int(node.x + node.y * airmatrix.indexRange[0] + node.z * airmatrix.indexRange[0] * airmatrix.indexRange[1])
    for neighbour in airmatrix.FindInNetwork(index):
        edges.append((node, airmatrix.FindInNodelist(neighbour[0])))

graph.add_edges_from(edges)

time_start = time.time()
edgeConnectivity = nx.edge_connectivity(graph)
time_end = time.time()
print('time cost', time_end-time_start, 's')