from PathFinder import AStar

class AircraftDataBase(object):
    def __init__(self):
        self.aircraftList = []
        self.aircraftList.append(AStar.Aircraft("DJI Mavic Air", 19, 4, 0.43, 27))
        self.aircraftList.append(AStar.Aircraft("DIY Drone", 12, 4, 0.3, 27))
        self.aircraftList.append(AStar.Aircraft("DJI Phantom 4", 20, 3, 1.375, 1))
        self.aircraftList.append(AStar.Aircraft("DJI Matrice 600 Pro", 18, 5, 10, 1))
        # self.aircraftList.append(AStar.Aircraft("VoloCity", 30, 6, 800, 1))