import sys
from Ant import Ant
from ACS import ACS
from Elitist import Elitist

class ACO:
    def __init__(self, problem, ants, iterations, pherInf, heurInf, algSpecific):
        self.problem = problem
        self.ants = int(ants)
        self.iterations = int(iterations)
        self.pherInf = float(pherInf)
        self.heurInf = float(heurInf)
        self.elitism = int(algSpecific[0])
        self.evap = float(algSpecifi[0])
        if (len(algSpecific) > 1):
            self.wearAway = algSpecific[1]
            self.tauZero = algSpecfic[2]
            self.acsProb = algSpecific[3]
        self.antGroup = []


    def buildAntGroup(self):
        for i in range(self.ants):
            a = Ant()
            self.antGroup += a

    def getCurBestLength(self):
        bestAntIndex = 0
        bestScore = 100000000
        for ant in self.antGroup:
            curTourLen = ant.getBestTour()
            if curTourLen < bestScore:
                bestAntIndex = self.antGroup.index(ant)
                bestScore = curTourLen
        return (bestScore, bestAntIndex)

# Get parameter input from the command line.
problem = sys.argv[1]
ants = sys.argv[2]
iterations = sys.argv[3]
pherInf = sys.argv[4]
heurInf = sys.argv[5]
if (len(sys.argv) == 7):
    algSpecific = (sys.argv[6])
elif(len(sys.argv) == 10):
    algSpecific = (sys.argv[6], sys.argv[7], sys.argv[8], sys.argv[9])
