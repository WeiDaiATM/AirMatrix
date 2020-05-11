from Matrix import MatrixBuilder, BuildingGenerator

airmatrix = MatrixBuilder.Matrix((3000, 3000, 1500), 100, 50)

obstacle = BuildingGenerator.BuildingGenerator()
obstacle.Generate(airmatrix, 100, 100, 10, 100, 10, 500, 10)
airmatrix.NodeListConstructor(obstacle)
airmatrix.MatrixConstructor()
