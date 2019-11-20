import math
import numpy as np
""" This class is used to compute the cost of a given solution"""
class Cost:
    #the get cost method takes a solution as a list of nodes
    def getCost(self,solution):
        #if it is given an empty solution it returns a very high distance
        #since we are trying to minimize cost
        if len(solution) == 0:
            return 1000000000000
        else:
            cost = 0
            previousNode = None
            #iterates through all nodes in solution and
            for node in solution:
                if previousNode == None:
                    previousNode = node
                else:
                    cost += self.getDistance(previousNode,node)
                    previousNode = node
            return cost

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

    def getBestPath(self,paths):
        lowestCost = 1000000000000
        bestPath = []
        for path in paths:
            cost = self.getCost(path)
            if cost < lowestCost:
                lowestCost = cost
                bestPath = path
        return bestPath

    def Lnn(self,problem):
        problem = list(problem)
        lenTour = 0
        tour = []
        unvisitedNodes = problem
        while len(tour) < len(problem):
            if len(tour) == 0:
                tour += [problem[0]]
                unvisitedNodes.remove(problem[0])
            else:
                currentNode = tour[-1]
                nextNode = [None]
                minDistance = 1000000000000
                for node in unvisitedNodes:
                    if nextNode[0] == None:
                        nextNode = node
                    else:
                        x1 = currentNode[1]
                        y1 = currentNode[2]
                        x2 = node[1]
                        y2 = node[2]
                        distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                        if distance < minDistance:
                            nextNode = node
                            minDistance = distance
                tour += [nextNode]
                lenTour += minDistance
                unvisitedNodes.remove(nextNode)
        return lenTour
