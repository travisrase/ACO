import math
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
                    x1 = previousNode[1]
                    y1 = previousNode[2]
                    x2 = node[1]
                    y2 = node[2]
                    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                    cost += distance
                    previousNode = node
            return cost
