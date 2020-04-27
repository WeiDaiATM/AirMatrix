
from PathFinder import TrafficPlan
from PathFinder import AStar
import random

class TrafficGenerator(object):
    def __init__(self, matrix, aircraftBase, earliestDepartureTime=0, latestDepartureTime=900, noFlights=20):
        self.trafficPlan = TrafficPlan.TrafficPlan()

        for i in range(noFlights):
            startPoint = AStar.Point((random.randint(0, matrix.indexRange[0]-1),
                                      random.randint(0, matrix.indexRange[1]-1), 0))
            endPoint = AStar.Point((random.randint(0, matrix.indexRange[0]-1),
                                    random.randint(0, matrix.indexRange[1]-1), 0))
            departureTime = random.randint(earliestDepartureTime, latestDepartureTime)
            aircraft = aircraftBase.aircraftList[random.randint(0, len(aircraftBase.aircraftList)-1)]
            plan = TrafficPlan.FlightPlan(startPoint, endPoint, departureTime, aircraft)
            self.trafficPlan.AppendFlightPlan(plan)

