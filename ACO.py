import numpy as np
import random
from Cost import Cost
class ACO:
    def __init__(self,algorithm, numAnts,numIter,alpha,beta,rho,elitismFactor, epsilon,tao0,q0,problem):
        #the type of algorithm to use (ACS or Elitist)
        self.isACS = False
        if algorithm == "a":
            self.isACS = True
        #number of ants in the colony
        self.numAnts = int(numAnts)
        #number of search iterations
        self.numIter = int(numIter)
        #the degree of influence of the phermones
        self.aplha = float(alpha)
        self.beta = float(beta)
        self.rho = float(rho)
        self.elitismFactor = float(elitismFactor)
        self.epsilon = float(epsilon)
        self.tao0 = float(tao0)
        self.q0 = float(q0)
        self.problem = problem
        #matrix keeping track of phermones between two nodes
        self.phermoneMatrix = []
        #a list of all ant solutions
        self.bestPath = []
        self.cost = Cost()
        self.ants =[]


    """TODO: We need to determine how the phermone matrix should be initialized.
    It is currently all 0's"""
    def initPhermoneMatrix(self):
        #init n x n 2d array where n = size of the problem
        self.phermoneMatrix = np.zeros((len(self.problem),len(self.problem)))

    def solve(self):
        #initialize the phermone matrix
        self.initPhermoneMatrix()
        i = 0
        while (i < self.numIter):
            paths = []
            for ant in range(self.numAnts):
                path = self.buildPath()
                self.ants += path
                paths += [path]
            self.bestPath = self.getBestPath(paths)
            self.updatePhermones(paths)

    #build a path for a given ant
    def buildPath(self):
        #the path created by a given ant
        path = []
        #the nodes that have not yet been visited by the ant
        unvisitedNodes = self.problem
        print(unvisitedNodes)
        for node in range(len(self.problem)):
            if len(path) > 0:
                currentNode = path[-1]
                print(currentNode)
                # PROBLEM WITH FILE? PROBLEM FOR FIRST TWO INDEX
                nextNode = self.getNextNode(unvisitedNodes,currentNode)
                print(nextNode)
            else:
                nextNode = unvisitedNodes[2]
            path += [nextNode]
            unvisitedNodes.remove(nextNode)
        if self.isACS:
            self.localPhermoneUpdate(path)
        return path

    #for a given point in a path, get the next node that the ant should visit
    def getNextNode(self,unvisitedNodes,currentNode=None):
        node = 0
        if currentNode == None:
            #randomly generate starting node index
            startIndex = random.randrange(0,len(self.problem))
            node = unvisitedNodes.remove(startIndex)
            return node
        else:
            #build ranges of proabilities for picking each node
            probs=[0]
            for unvisitedNode in unvisitedNodes:
                t = self.getPhermone(currentNode,unvisitedNode)
                distance = self.getDistance(currentNode,unvisitedNode)
                val = t**self.alpha * (1/distance)**self.beta
                probs += [val]
            #normalizeRanges so all values are between 0 and 1
            sum = sum(probs)
            probs += [i/sum for i in probs]
            node = np.random.choice(unvisitedNodes, 1, p=probs)
            return node

    def getPhermone(self,node1,node2):
        a = node1[0]
        b = node2[0]
        return self.phermoneMatrix[a,b]

    def setPhermone(self,node1,node2,val):
        a = node1[0]
        b = node2[0]
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
        # Loop through pheremone matrix
        #Get value at index of row and col
        for row in self.phermoneMatrix:
            for col in self.phermoneMatrix:
                #Get the corresponing nodes for given path
                node1 = [x[0] for x in paths].index(row)
                node2 = [x[0] for x in paths].index(col)
                #Apply pheremone update rule according to Elitism
                updateValue = (1-self.rho)*self.phermoneMatrix[row][col] + (1/self.getDistance(node1, node2)) + self.elitismFactor*(self.bestPath)
                # Update pheremone matrix
                self.phermoneMatrix[row][col] = updateValue

    def updatePhermonesACS(self,paths):
        bestPath = self.getBestPath(paths)
        costBestPath = self.cost.getCost(bestPath)
        previousNode = None
        for node in bestPath:
            if previousNode == None:
                previousNode = node
            else:
                tCurrent = self.getPhermone(previousNode,node)
                t = (1-self.rho)*tCurrent + self.rho*self.cost.getCost(bestPath)
                self.setPhermone(previousNode,node,t)

    def getBestPath(self,paths):
        lowestCost = 1000000000000
        bestPath = []
        for path in paths:
            cost = self.cost.getCost(path)
            if cost < lowestCost:
                lowestCost = cost
                bestPath = path
        return bestPath

    #helpers
    def getDistance(self,node1,node2):
        x1 = node1[1]
        y1 = node1[2]
        x2 = node2[1]
        y2 = node2[2]
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return distance
