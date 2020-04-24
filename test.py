
from Matrix import MatrixBuilder
from PathFinder import AStar
from PathFinder import AStarwObstacle

airmatrix = MatrixBuilder.Matrix((3000, 3000, 1000))
airmatrix.MatrixConstructor()

dji = AStar.Aircraft()
startPoint = AStar.Point((1, 2, 0))
endPoint = AStar.Point((25, 25, 0))

finder = AStar.AStarClassic(airmatrix, startPoint, endPoint, dji)
result = finder.Search()
