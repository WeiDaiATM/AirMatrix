from PathFinder import AStar

class AircraftDataBase(object):
    def __init__(self):
        self.aircraftList = []
        self.aircraftList.append(AStar.Aircraft())