import numpy as np
import random
from Cost import Cost
import math
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
        self.alpha = float(alpha)
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
        self.phermoneMatrix = np.full((len(self.problem),len(self.problem)), self.tao0)

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
            if self.cost.getCost(self.getBestPath(paths)) < self.cost.getCost(self.bestPath) or len(self.bestPath) == 0:
                self.bestPath = self.getBestPath(paths)
            self.updatePhermones(paths)
            i += 1
            print("i: ", i)
            print("cost: " , self.cost.getCost(self.bestPath))
        return self.bestPath,self.cost.getCost(self.bestPath)

    #build a path for a given ant
    def buildPath(self):
        #the path created by a given ant
        path = []
        #the nodes that have not yet been visited by the ant
        unvisitedNodes = self.problem[:]
        for node in range(len(self.problem)):
            if len(path) > 0:
                currentNode = path[-1]
                # PROBLEM WITH FILE? PROBLEM FOR FIRST TWO INDEX
                nextNode = self.getNextNode(unvisitedNodes,currentNode)
                if nextNode == None:
                    break
            else:
                nextNode = unvisitedNodes[0]
            path += [nextNode]
            unvisitedNodes.remove(nextNode)
        if self.isACS:
            self.localPhermoneUpdate(path)
        return path

    #for a given point in a path, get the next node that the ant should visit
    def getNextNode(self,unvisitedNodes,currentNode=None):
        node = 0
        sumProbs = 0
        if len(unvisitedNodes) == 0:
            return None
        if currentNode == None:
            #randomly generate starting node index
            startIndex = random.randrange(0,len(self.problem))
            node = unvisitedNodes(startIndex)
            return node
        else:
            if self.isACS:
                r = random.random()
                if r < self.q0:
                    bestNode = None
                    bestVal = 0
                    previousNode = None
                    for unvisitedNode in unvisitedNodes:
                        if bestNode == None:
                            bestNode = unvisitedNode
                        else:
                            val = self.getPhermone(previousNode,unvisitedNode)*(1/self.getDistance(currentNode,unvisitedNode))**self.beta
                            if val > bestVal:
                                bestVal = val
                                bestNode = unvisitedNode
                        previousNode = unvisitedNode
                    return bestNode
            #build ranges of proabilities for picking each node
            probs=[]
            for unvisitedNode in unvisitedNodes:
                t = self.getPhermone(currentNode,unvisitedNode)
                distance = self.getDistance(currentNode,unvisitedNode)
                if distance != 0:
                    val = t**self.alpha * (1/distance)**self.beta
                else:
                    val = 0
                probs += [val]
            #normalizeRanges so all values are between 0 and 1
            sumProbs = sum(probs)
            probs = [abs(i/sumProbs) for i in probs]
            indexList = np.arange(0,len(unvisitedNodes),1)
            index = np.random.choice(indexList,  p=probs)
            return unvisitedNodes[index]

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
        # Loop through pheremone matrix
        #Get value at index of row and col
        for row in self.phermoneMatrix:
            for col in self.phermoneMatrix:
                #Get the corresponing nodes for given path
                node1 = [x[0] for x in paths].index(row)
                node2 = [x[0] for x in paths].index(col)
                #Apply pheremone update rule according to Elitism
                updateValue = (1-self.rho)*self.getPhermone(row,col) + (1/self.getDistance(node1, node2)) + self.elitismFactor*(self.bestPath)
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
        try:
            x1 = node1[1]
            y1 = node1[2]
            x2 = node2[1]
            y2 = node2[2]
            distance = math.sqrt(abs((x2-x1)**2 + (y2-y1)**2))
            if distance == 0:
                return 0.00001
            else:
                return distance
        except:
            return 0.00001
