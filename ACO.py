import numpy as np
import random
from Cost import Cost
import math
from Phermone import Phermone
class ACO:
    def __init__(self,algorithm, numAnts,numIter,problem,optimalSolutionLenth,tolerance):
        #the type of algorithm to use (ACS or Elitist)
        self.isACS = False
        if algorithm == "a":
            self.isACS = True
        #number of ants in the colony
        self.numAnts = int(numAnts)
        #number of search iterations
        self.numIter = int(numIter)
        #the degree of influence of the phermones
        self.alpha = 2.0
        #the degree of influnece of the hueristic component
        self.beta = float(4)
        #probability that the ant will choose the best leg for the next leg of the tour
        self.q0 = float(.85)
        self.problem = problem
        #the length of the known best solution
        self.optimalSolutionLenth = float(optimalSolutionLenth)
        #the tolerance for how close our solution is to the best
        self.tolerance = float(tolerance)
        #matrix keeping track of phermones between two nodes
        self.Phermones = Phermone(algorithm,problem)
        self.cost = Cost()
        #a list of all ant solutions
        self.bestPath = []
        self.bestPathCost = 100000000000
        self.iterationsSinceCostUpdate = 0

    def solve(self):
        #initialize the phermone matrix
        self.Phermones.initPhermones()
        i = 0
        #make sure i < numIter
        while (i < self.numIter and self.optimalSolutionLenth/self.cost.getCost(self.bestPath) < self.tolerance):
            #store paths created at every iteration
            paths = []
            #for each ant build a path
            for ant in range(self.numAnts):
                #build a path
                path = self.buildPath()
                paths += [path]
            #update best path and best path cost
            localBestPath = self.cost.getBestPath(paths)
            localBestPathCost = self.cost.getCost(localBestPath)
            if localBestPathCost < self.bestPathCost or len(self.bestPath) == 0:
                self.bestPath = self.cost.getBestPath(paths)
                self.bestPathCost = localBestPathCost
                self.iterationsSinceCostUpdate = 0
            else:
                self.iterationsSinceCostUpdate += 1
            #check to see how many iterations since the minimum value found has been updated
            if (self.iterationsSinceCostUpdate >= 50):
                break
            #update phermones
            self.Phermones.updatePhermones(paths)
            #update iterations
            i += 1
            print("i: ", i)
            print("Best Path Cost: ", self.bestPathCost)
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
                #randomly generate the index for the first node
                index = random.randrange(0,len(self.problem))
                nextNode = unvisitedNodes[index]
            #add node to the path
            path += [nextNode]
            #remove selected node from the unvisted set of nodes
            unvisitedNodes.remove(nextNode)
        #if it is ACS then do the local phermone update after each path is created
        if self.isACS:
            self.Phermones.localPhermoneUpdate(path)
        return path

    #for a given point in a path, get the next node that the ant should visit
    def getNextNode(self,unvisitedNodes,currentNode=None):
        #make sure that there are unvisted nodes in the list
        if len(unvisitedNodes) == 0:
            return None
        #Check to see if it is running ACS
        if self.isACS:
            r = random.random()
            #with some probability select the next node greedily
            if r < self.q0:
                return self.getNextNodeGreedy(unvisitedNodes,currentNode)
        #build ranges of proabilities for picking each node
        probabilities = self.getProbabilities(currentNode, unvisitedNodes)
        indexList = np.arange(0,len(unvisitedNodes),1)
        index = np.random.choice(indexList, p=probabilities)
        return unvisitedNodes[index]

    def getNextNodeGreedy(self,unvisitedNodes,currentNode):
        bestNode = None
        bestVal = 0
        #previousNode = None
        for unvisitedNode in unvisitedNodes:
            if bestNode == None:
                t = self.Phermones.getPhermone(currentNode,unvisitedNode)
                distance = self.cost.getDistance(currentNode,unvisitedNode)
                bestVal = t*(1/distance)**self.beta
            else:
                t = self.Phermones.getPhermone(currentNode,unvisitedNode)
                distance = self.cost.getDistance(currentNode,unvisitedNode)
                val = t*(1/distance)**self.beta
                if val > bestVal:
                    bestVal = val
                    bestNode = unvisitedNode
        return bestNode

    def getProbabilities(self,currentNode, unvisitedNodes):
        probs=[]
        for unvisitedNode in unvisitedNodes:
            phermone = self.Phermones.getPhermone(currentNode,unvisitedNode)
            distance = self.cost.getDistance(currentNode,unvisitedNode)
            if distance != 0:
                val = phermone**self.alpha * (1/distance)**self.beta
            else:
                val = 0
            probs += [val]
        #normalizeRanges so all values are between 0 and 1
        sumProbs = sum(probs)
        probs = [abs(i/sumProbs) for i in probs]
        return probs
