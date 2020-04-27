from PathFinder import AStar

class AircraftDataBase(object):
    def __init__(self):
        self.aircraftList = []
        self.aircraftList.append(AStar.Aircraft(18, 5, 0.5))
        self.aircraftList.append(AStar.Aircraft(25, 5, 4))
        self.aircraftList.append(AStar.Aircraft(12, 4, 0.3))
        self.aircraftList.append(AStar.Aircraft(30, 6, 15))
        self.aircraftList.append(AStar.Aircraft(20, 4.5, 3))