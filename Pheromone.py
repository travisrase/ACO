import numpy as np
from Cost import Cost
class Pheromone:
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
        self.pheromoneMatrix = []
        self.bestPathMatrix = []
        self.cost = Cost()

    def initPheromones(self):
        #initialize pheromones as tao0
        self.tao0 = 1/((self.cost.Lnn(self.problem)) * self.lenProblem)
        #init n x n 2d array where n = size of the problem
        self.pheromoneMatrix = np.full((self.lenProblem,self.lenProblem), self.tao0)

    #getter method for pheromones, 0 based indexing
    def getPheromone(self,node1,node2):
        a = node1[0]-1
        b = node2[0]-1
        return self.pheromoneMatrix[a,b]

    #setter method for pheromones, 0 based indexing
    def setPheromone(self,node1,node2,val):
        a = node1[0]-1
        b = node2[0]-1
        self.pheromoneMatrix[a,b] = val
        self.pheromoneMatrix[b,a] = val

    #update pheromones based on the algorithm selected
    def updatePheromones(self,paths):
        if (self.isACS):
            self.updatePheromonesACS(paths)
        else:
            self.updatePheromonesElitist(paths)

    #local pheromone update used only for ACS
    def localPheromoneUpdate(self,path):
        #wears away pheromones on edges travled by ants
        previousNode = None
        for node in path:
            if previousNode == None:
                previousNode = node
            else:
                #get the current pheromone value
                tCurrent = self.getPheromone(previousNode,node)
                #this will not change anything in most cases
                t = (1-self.epsilon)*tCurrent + self.epsilon*self.tao0
                self.setPheromone(previousNode,node,t)
                previousNode = node

    def updatePheromonesElitist(self,paths):
        #get the current best path (historic)
        bestPath = self.cost.getBestPath(paths)
        #best path matrix used to identify edges in best path
        self.bestPathMatrix = self.buildPathMatrix(bestPath)
        #evaporate all edges
        self.pheromoneMatrix = self.pheromoneMatrix * (1-self.rho)
        #update edges that are in a given path and the best path
        for path in paths:
            distance = self.cost.getCost(path)
            previousNode = path[0]
            for node in path:
                if node == previousNode:
                    continue
                else:
                    #get current pheromone value
                    currentPheromone = self.getPheromone(previousNode, node)
                    #potenital new pheromone value
                    newPheromoneValue = currentPheromone + 1/distance
                    eliteFactor = 0
                    if self.bestPathMatrix[node[0]-1][previousNode[0]-1] != 0:
                        #if edge in best path multiply by elitism factor
                        eliteFactor = self.getPheromone(node,previousNode) * self.elitismFactor
                    newPheromoneValue += eliteFactor
                    self.setPheromone(previousNode,node,newPheromoneValue)

        #update edges in best path
        bestPathCost = self.cost.getCost(bestPath)
        previousNode = bestPath[0]
        for node in bestPath:
            if node == previousNode:
                continue
            else:
                currentPheromone = self.getPheromone(previousNode, node)
                newPheromoneValue = currentPheromone + (1/bestPathCost) * self.elitismFactor
                self.setPheromone(previousNode,node,newPheromoneValue)

    def updatePheromonesACS(self,paths):
        #only update edges in best path
        bestPath = self.cost.getBestPath(paths)
        costBestPath = self.cost.getCost(bestPath)
        previousNode = None
        for node in bestPath:
            if previousNode == None:
                previousNode = node
            else:
                #compute updated value for pheromone
                tCurrent = self.getPheromone(previousNode,node)
                pheromone = (1-self.rho)*tCurrent + self.rho*(1/self.cost.getCost(bestPath))
                self.setPheromone(previousNode,node,pheromone)

    #used to efficiently check whether or not an edge is in the best path
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
