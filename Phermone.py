import numpy as np
from Cost import Cost
class Phermone:
    def __init__(self,algorithm,problem):
        #the type of algorithm to use (ACS or Elitist)
        self.isACS = False
        if algorithm == "a":
            self.isACS = True
        self.lenProblem = len(problem)
        self.problem = problem
        self.tao0 = 0
        self.epsilon = float(.1)
        self.rho = float(.2)
        self.elitismFactor = float(1)
        self.phermoneMatrix = []
        self.bestPathMatrix = []
        self.cost = Cost()

    def initPhermones(self):
        self.tao0 = 1/((self.cost.Lnn(self.problem)) * self.lenProblem)
        #init n x n 2d array where n = size of the problem
        self.phermoneMatrix = np.full((self.lenProblem,self.lenProblem), self.tao0)

    def getPhermone(self,node1,node2):
        a = node1[0]-1
        b = node2[0]-1
        return self.phermoneMatrix[a,b]

    def setPhermone(self,node1,node2,val):
        a = node1[0]-1
        b = node2[0]-1
        self.phermoneMatrix[a,b] = val
        self.phermoneMatrix[b,a] = val

    def updatePhermones(self,paths):
        if (self.isACS):
            self.updatePhermonesACS(paths)
        else:
            self.updatePhermonesElitist(paths)

    def localPhermoneUpdate(self,path):
        previousNode = None
        for node in path:
            if previousNode == None:
                previousNode = node
            else:
                tCurrent = self.getPhermone(previousNode,node)
                t = (1-self.epsilon)*tCurrent + self.epsilon * self.tao0
                self.setPhermone(previousNode,node,t)
                previousNode = node

    def updatePhermonesElitist(self,paths):
        bestPath = self.cost.getBestPath(paths)
        #evaporate all edges
        self.phermoneMatrix = self.phermoneMatrix * (1-self.rho)
        for path in paths:
            distance = self.cost.getCost(path)
            previousNode = path[0]
            for node in path:
                if node == previousNode:
                    continue
                else:
                    currentPhermone = self.getPhermone(previousNode, node)
                    newPhermoneValue = currentPhermone + 1/distance
                    eliteFactor = 0
                    if self.bestPathMatrix[previousNode[0]][node[0]] != 0:
                        eliteFactor = self.bestPathMatrix[previousNode[0]][node[0]] * self.elitismFactor
                    newPhermoneValue += eliteFactor
                    self.setPhermone(previousNode,node,newPhermoneValue)

        bestPathCost = self.cost.getCost(bestPath)
        previousNode = bestPath[0]
        for node in bestPath:
            if node == previousNode:
                continue
            else:
                print("BEST PATH HIT")
                currentPhermone = self.getPhermone(previousNode, node)
                newPhermoneValue = currentPhermone + (1/bestPathCost) * self.elitismFactor
                self.setPhermone(previousNode,node,newPhermoneValue)

    def updatePhermonesACS(self,paths):
        bestPath = self.cost.getBestPath(paths)
        costBestPath = self.cost.getCost(bestPath)
        previousNode = None
        for node in bestPath:
            if previousNode == None:
                previousNode = node
            else:
                tCurrent = self.getPhermone(previousNode,node)
                t = (1-self.rho)*tCurrent + self.rho*(1/self.cost.getCost(bestPath))
                self.setPhermone(previousNode,node,t)

    def buildPathMatrix(self,path):
        lengthPath = len(path)
        matrix = np.zeros((lengthPath,lengthPath))
        previousNode = path[0]
        for node in path:
            if node == previousNode:
                continue
            else:
                index1 = previousNode[0]
                index2 = node[0]
                matrix[index1-1][index2-1] = 1

        #print("Path MAtrix: ",len(matrix))
        return matrix
