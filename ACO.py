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
        self.problem = []
        #matrix keeping track of phermones between two nodes
        self.phermoneMatrix = []
        #a list of all ant solutions
        self.bestPath = []
        self.cost = Cost()

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
                paths += [path]
            self.updatePhermones(paths)


    #build a path for a given ant
    def buildPath(self):
        #the path created by a given ant
        path = []
        #the nodes that have not yet been visited by the ant
        unvisitedNodes = self.problem
        for node in range(len(self.problem)):
            currentNode = path[-1]
            nextNode = self.getNextNode(unvisitedNodes,currentNode)
            path += [nextNode]
            unvisitedNodes.remove(nextNode)
        if self.isACS:

        return path

    #for a given point in a path, get the next node that the ant should visit
    def getNextNode(self,unvisitedNodes,currentNode=None):
        node = 0
        if currentNode == None:
            #randomly generate starting node index
            startIndex = random.randrange(0,len(self.problem))
            node = unvisitedNodes.remove(startIndex)
        else:
            #ACS
            if (self.algorithm == "a"):
                node = self.getNextNodeACS(currentNode,unvisitedNodes)
            #Elitist
            else:
                node = self.getNextNodeElitist(currentNode,unvisitedNodes)
        return node

    def getNextNodeElitist(self,path,unvisitedNodes):
        #init probabilities list
        probabilitiesList = []
        # all unvisited nodes
        #Get all unvisited node Sum
        for unvisited in unvisitedNodes:
            # Get the pheramone concetration
            phermone = self.pheremoneMatrix[path[0]][node[0]]
           # Calc distance between node i and node j
            dist = self.cost.getCost(path, node)
            # Probability update divisor
            sumUnvisited += (pow(phermone, self.alpha) * pow(1/dist, self.beta))
       # Loop through again to get the probabilities for each unvisited node
        for node in unvisitedNodes:
            phermone = self.pheremoneMatrix[path[0]][node[0]]
            dist = self.cost.getCost(path, node)
            # Get the probability of the unvisited node being chosen according to elite rule
            nodeProbability = (pow(phermone, self.alpha) * pow(1/dist, self.beta))/ sumUnvisited
           # Get tuple containing node index and the probability of being chosen
            nodeTuple = (node[0], nodeProbability)
            # Add probability tuple to the list of probabilities
            probabilitiesList.append(nodeTuple)
        return probabilitiesList

    def getNextNodeACS(self,currentNode,unvisitedNodes):
        #build ranges of proabilities for picking each node
        probabilityRanges=[0]
        for unvisitedNode in unvisitedNodes:
            t = self.getPhermone(currentNode,unvisitedNode)
            distance = self.getDistance(currentNode,unvisitedNode)
            val = t * (1/distance)
            probabilityRanges += [probabilityRanges[-1] + val]

        #normalizeRanges so all values are between 0 and 1
        sum = sum(probabilityRanges)
        normalizedRanges = [i/sum for i in probabilityRanges]
        #generate random number between 0 and 1
        r = random.random()
        #iterate through ranges to see where the value falls
        for j in range(normalizedRanges):
            if r > normalizedRanges[j]:
                continue
            else:
                #range found, return the unvisited node at j-1
                #this node corresponds with range associated
                #with the random number found
                return unvisitedNodes[j-1]

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
        if (self.algorithm == "a"):
            self.updatePhermonesACS(paths)
        else:
            self.updatePhermonesElitist(paths)


    def updatePhermonesElitist(self,paths):
        print()

    def updatePhermonesACS(self,paths):
        print()

    #helpers
    def getDistance(self,node1,node2):
        x1 = node1[1]
        y1 = node1[2]
        x2 = node2[1]
        y2 = node2[2]
        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return distance
