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
        self.rho = float(.05)
        self.elitismFactor = float(10)
        self.phermoneMatrix = []
        self.bestPathMatrix = []
        self.cost = Cost()

    def initPhermones(self):
        #initialize phermones as tao0
        self.tao0 = 1/((self.cost.Lnn(self.problem)) * self.lenProblem)
        #init n x n 2d array where n = size of the problem
        self.phermoneMatrix = np.full((self.lenProblem,self.lenProblem), self.tao0)

    #getter method for phermones, 0 based indexing
    def getPhermone(self,node1,node2):
        a = node1[0]-1
        b = node2[0]-1
        return self.phermoneMatrix[a,b]

    #setter method for phermones, 0 based indexing
    def setPhermone(self,node1,node2,val):
        a = node1[0]-1
        b = node2[0]-1
        self.phermoneMatrix[a,b] = val
        self.phermoneMatrix[b,a] = val

    #update phermones based on the algorithm selected
    def updatePhermones(self,paths):
        if (self.isACS):
            self.updatePhermonesACS(paths)
        else:
            self.updatePhermonesElitist(paths)

    #local phermone update used only for ACS
    def localPhermoneUpdate(self,path):
        #wears away phermones on edges travled by ants
        previousNode = None
        for node in path:
            if previousNode == None:
                previousNode = node
            else:
                #get the current phermone value
                tCurrent = self.getPhermone(previousNode,node)
                #this will not change anything in most cases
                t = (1-self.epsilon)*tCurrent + self.epsilon*self.tao0
                self.setPhermone(previousNode,node,t)
                previousNode = node

    def updatePhermonesElitist(self,paths):
        #get the current best path (historic)
        bestPath = self.cost.getBestPath(paths)
        #best path matrix used to identify edges in best path
        self.bestPathMatrix = self.buildPathMatrix(bestPath)
        #evaporate all edges
        self.phermoneMatrix = self.phermoneMatrix * (1-self.rho)
        #update edges that are in a given path and the best path
        for path in paths:
            distance = self.cost.getCost(path)
            previousNode = path[0]
            for node in path:
                if node == previousNode:
                    continue
                else:
                    #get current phermone value
                    currentPhermone = self.getPhermone(previousNode, node)
                    #potenital new phermone value
                    newPhermoneValue = currentPhermone + 1/distance
                    eliteFactor = 0
                    if self.bestPathMatrix[node[0]-1][previousNode[0]-1] != 0:
                        #if edge in best path multiply by elitism factor
                        eliteFactor = self.getPhermone(node,previousNode) * self.elitismFactor
                    newPhermoneValue += eliteFactor
                    self.setPhermone(previousNode,node,newPhermoneValue)

        #update edges in best path
        bestPathCost = self.cost.getCost(bestPath)
        previousNode = bestPath[0]
        for node in bestPath:
            if node == previousNode:
                continue
            else:
                currentPhermone = self.getPhermone(previousNode, node)
                newPhermoneValue = currentPhermone + (1/bestPathCost) * self.elitismFactor
                self.setPhermone(previousNode,node,newPhermoneValue)

    def updatePhermonesACS(self,paths):
        #only update edges in best path
        bestPath = self.cost.getBestPath(paths)
        costBestPath = self.cost.getCost(bestPath)
        previousNode = None
        for node in bestPath:
            if previousNode == None:
                previousNode = node
            else:
                #compute updated value for phermone
                tCurrent = self.getPhermone(previousNode,node)
                phermone = (1-self.rho)*tCurrent + self.rho*(1/self.cost.getCost(bestPath))
                self.setPhermone(previousNode,node,phermone)

    #used to efficiently check weather or not an edge is in the best path
    def buildPathMatrix(self,path):
        #build a matrix with 1's on edges in best path
        lengthPath = len(path)
        matrix = np.zeros((lengthPath,lengthPath))
        previousNode = path[0]
        for node in path:
            if node == previousNode:
                continue
            else:
                index1 = previousNode[0]
                index2 = node[0]
                #set matrix index to 1.
                matrix[index1-1][index2-1] = 1
                matrix[index2-1][index1-1] = 1
        return matrix
