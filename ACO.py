import numpy as np
import random
from Cost import Cost
import math
from Pheromone import Pheromone
class ACO:
    def __init__(self,algorithm, numAnts,numIter,problem,optimalSolutionLength,tolerance):
        #the type of algorithm to use (ACS or Elitist)
        self.isACS = False
        if algorithm == "a":
            self.isACS = True
        #number of ants in the colony
        self.numAnts = int(numAnts)
        #number of search iterations
        self.numIter = int(numIter)
        #the degree of influence of the pheromones
        self.alpha = float(1.0)
        #the degree of influnece of the hueristic component
        self.beta = float(30.0)
        #probability that the ant will choose the best leg for the next leg of the tour
        self.q0 = float(.90)
        self.problem = problem
        #the length of the known best solution
        self.optimalSolutionLength = float(optimalSolutionLength)
        #the tolerance for how close our solution is to the best
        self.tolerance = float(tolerance)
        #matrix keeping track of pheromones between two nodes
        self.Pheromones = Pheromone(algorithm,problem)
        self.cost = Cost()
        #a list of all ant solutions
        self.bestPath = []
        self.bestPathCost = 100000000000
        self.iterationsSinceCostUpdate = 0
        self.optPathCost = []

    def solve(self):
        #initialize the pheromone matrix
        self.Pheromones.initPheromones()
        i = 0
        #make sure i < numIter
        while (i < self.numIter and self.optimalSolutionLength/self.cost.getCost(self.bestPath) < self.tolerance):
            #store paths created at every iteration
            paths = []
            #for each ant build a path
            for ant in range(self.numAnts):
                #build a path
                path = self.buildPath()
                paths += [path]
            #update best path and best path cost
            newBestPath = self.cost.getBestPath(paths)
            newBestPathCost = self.cost.getCost(newBestPath)
            if newBestPathCost < self.bestPathCost or len(self.bestPath) == 0:
                self.bestPath = newBestPath
                self.bestPathCost = newBestPathCost
                self.iterationsSinceCostUpdate = 0
            else:
                self.iterationsSinceCostUpdate += 1
            #check to see how many iterations since the minimum value found has been updated
            if (self.iterationsSinceCostUpdate >= 200):
                break
            #update pheromones
            self.Pheromones.updatePheromones(paths)
            #update iterations
            i += 1
            print("i: ", i)
            print("Best Path Cost: ", self.bestPathCost)
            self.optPathCost += [self.bestPathCost]
        return self.bestPath,self.cost.getCost(self.bestPath)

    def optPathCost(self):
        return self.optPathCost

    #build a path for a given ant
    def buildPath(self):
        #the path created by a given ant
        path = []
        #the nodes that have not yet been visited by the ant
        unvisitedNodes = self.problem[:]
        for node in range(len(self.problem)):
            if len(path) > 0:
                currentNode = path[-1]
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
        #if it is ACS then do the local pheromone update after each path is created
        if self.isACS:
            self.Pheromones.localPheromoneUpdate(path)
        return path

    #for a given point in a path, get the next node that the ant should visit
    def getNextNode(self,unvisitedNodes,currentNode):
        #make sure that there are unvisted nodes in the list
        if len(unvisitedNodes) == 0:
            return None
        #Check to see if it is running ACS
        if self.isACS:
            r = random.random()
            #with some probability select the next node greedily
            if r < self.q0:
                return self.getNextNodeGreedy(currentNode,unvisitedNodes)
        #build ranges of proabilities for picking each node
        probabilities = self.getProbabilities(currentNode, unvisitedNodes)
        indexList = np.arange(0,len(unvisitedNodes),1)
        index = np.random.choice(indexList, p=probabilities)
        return unvisitedNodes[index]

    #get the next node greedily
    def getNextNodeGreedy(self,currentNode,unvisitedNodes):
        #look for the best node and return that node
        bestNode = None
        bestVal = 0
        #look through all unvisited nodes
        for unvisitedNode in unvisitedNodes:
            if bestNode == None:
                t = self.Pheromones.getPheromone(currentNode,unvisitedNode)
                distance = self.cost.getDistance(currentNode,unvisitedNode)
                bestVal = t*(1/distance)**self.beta
            else:
                #get the current pheromone value between current node and unvisted node
                pheromone = self.Pheromones.getPheromone(currentNode,unvisitedNode)
                #get the distance between current node and unvisted node
                distance = self.cost.getDistance(currentNode,unvisitedNode)
                val = pheromone*(1/distance)**self.beta
                #if the computed value is greater than the previous bestVal
                #update best val
                if val > bestVal:
                    bestVal = val
                    bestNode = unvisitedNode
        return bestNode

    #this method computes the probabilities to choose the next node
    def getProbabilities(self,currentNode, unvisitedNodes):
        probs=[]
        for unvisitedNode in unvisitedNodes:
            #get current pheromone value
            pheromone = self.Pheromones.getPheromone(currentNode,unvisitedNode)
            #get distance
            distance = self.cost.getDistance(currentNode,unvisitedNode)
            if distance != 0:
                #compute the approximate value of the path
                val = pheromone**self.alpha * (1/distance)**self.beta
            else:
                val = 0
            probs += [val]
        #normalizeRanges so all values are between 0 and 1
        sumProbs = sum(probs)
        probs = [abs(i/sumProbs) for i in probs]
        return probs
