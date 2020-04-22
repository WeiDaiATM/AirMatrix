class FlightPlan(object):
    def __init__(self, startPoint, endPoint, departureTime, aircraft):
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.departureTime = departureTime
        self.aircraft = aircraft


class TrafficPlan(object):
    def __init__(self):
        self.scheduledFlights = list()

    def AppendFlightPlan(self, flightPlan, priority=None):
        if not priority:
            self.scheduledFlights.append(flightPlan)
            return 0
        else: # to be developed later
            None