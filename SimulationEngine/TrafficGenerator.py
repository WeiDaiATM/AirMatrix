
from PathFinder import TrafficPlan
from PathFinder import AStar
import random

class TrafficGenerator(object):
    def __init__(self, matrix, aircraftBase, noFlights=20, earliestDepartureTime=0, latestDepartureTime=900):
        self.trafficPlan = TrafficPlan.TrafficPlan()

        startPoints = list()
        for j in range(matrix.indexRange[0]):
            for k in range(matrix.indexRange[1]):
                startPoints.append(AStar.Point((j,k,0)))
                # for l in range(matrix.indexRange[2]):
                #     startPoints.append(AStar.Point((j, k, l)))

        self.odPairs = random.sample(startPoints, 2*noFlights)

        for i in range(noFlights):
            startPoint = self.odPairs[i*2]
            endPoint = self.odPairs[i*2+1]
            # startPoint = AStar.Point((random.randint(0, matrix.indexRange[0]-1),
            #                           random.randint(0, matrix.indexRange[1]-1), 0))
            # endPoint = AStar.Point((random.randint(0, matrix.indexRange[0]-1),
            #                         random.randint(0, matrix.indexRange[1]-1), 0))
            departureTime = random.randint(earliestDepartureTime, latestDepartureTime)
            aircraft = aircraftBase.aircraftList[random.randint(0, len(aircraftBase.aircraftList)-1)]
            plan = TrafficPlan.FlightPlan(startPoint, endPoint, departureTime, aircraft)
            self.trafficPlan.AppendFlightPlan(plan)


