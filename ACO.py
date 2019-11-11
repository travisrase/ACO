import numpy as np
import random
class ACO:
    def __init__(self,algorithm, numAnts,numIter,alpha,beta,rho,elitismFactor, epsilon,tao0,q0,problem):
        #the type of algorithm to use (ACS or Elitist)
        self.algorithm = algorithm
        #number of ants in the colony
        self.numAnts = numAnts
        #number of search iterations
        self.numIter = numIter
        #the degree of influence of the phermones
        self.aplha = alpha
        self.beta = beta
        self.rho = rho
        self.elitismFactor = elitismFactor
        self.epsilon = epsilon
        self.tao0 = tao0
        self.q0 = q0
        self.problem = []
        #matrix keeping track of phermones between two nodes
        self.permoneMatrix = []
        #a list of all ant solutions
        self.bestPath = []

    def initPhermoneMatrix(self):
        #init n x n 2d array where n = size of the problem
        self.permoneMatrix = np.zeros[(len(self.problem),(len(self.problem))]

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
            nextNode = self.getNextNode(currentNode,unvisitedNodes)
            path += [nextNode]
            unvisitedNodes.remove(nextNode)
        return path

    #for a given point in a path, get the next node that the ant should visit
    def getNextNode(self,currentNode=None,unvisitedNodes):
        node = 0
        if currentNode == None:
            #randomly generate starting node index
            startIndex = random.randrange(0,len(self.problem))
            node = unvisitedNodes.remove(startIndex)
        else:
            #ACS
            if (self.algorithm == "a"):
                node = self.getNextNodeACS(path,unvisitedNodes)
            #Elitist
            else:
                node = self.getNextNodeElitist(path,unvisitedNodes)
        return node

    def getNextNodeElitist(self,path,unvisitedNodes):
        print()

    def getNextNodeACS(self,path,unvisitedNodes):
        print()

    def updatePhermones(self,paths)
        print()
        self.updatePhermonesElitist(paths)
        self.updatePhermonesACS(paths)

    def updatePhermonesElitist(self,paths)
        print()

    def updatePhermonesACS(self,paths)
        print()
