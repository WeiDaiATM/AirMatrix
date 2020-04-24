
from PathFinder import TrafficPlan
from PathFinder import AStar
import random

class TrafficGenerator(object):
    def __init__(self, matrix, aircraftList, earliestDepartureTime=0, latestDepartureTime=900, noFlights=20):
        self.trafficPlan = TrafficPlan.TrafficPlan()

        for i in range(noFlights):
            startPoint = AStar.Point((random.randint(0,matrix.indexRange[0]),random.randint(0,matrix.indexRange[1]),0))
            endPoint = AStar.Point((random.randint(0,matrix.indexRange[0]),random.randint(0,matrix.indexRange[1]),0))
            departureTime = random.randint(earliestDepartureTime,latestDepartureTime)
            aircraft = aircraftList[random.randint(0,len(aircraftList))]
            plan = TrafficPlan.FlightPlan(startPoint, endPoint, departureTime, aircraft)
            self.trafficPlan.scheduledFlights(plan)

