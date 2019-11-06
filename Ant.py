class Ant:
    def __init__(self, probability, townsVisited, townsToVisit):
        self.probability = probability
        self.townsVisited = townsVisited
        self.townsToVisit = townsToVisit
        self.bestTour = 100000000

    def getBestTour(self):
        return self.bestTour
