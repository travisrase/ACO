import math
class Cost:
    def getCost(self,solution):
        if len(solution) == 0:
            return 1000000000000
        else:
            cost = 0
            previousNode = None
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
