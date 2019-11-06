import sys
from Ant import Ant
from ACS import ACS
from Elitist import Elitist

class ACO:
    def __init__(self, ants, iterations, pherInf, heurInf, algSpecific):
        self.ants = int(ants)
        self.iterations = int(iterations)
        self.pherInf = float(pherInf)
        self.heurInf = float(heurInf)
        self.elitism = int(algSpecific[0])
        self.evap = float(algSpecifi[0])
        self.acsProb = 0.9
        self.antGroup = []

    def buildAntGroup(self):
