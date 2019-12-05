import sys
from ACO import ACO
import time
class Optimize:
    def __init__(self,algorithm,numAnts,numIter,problemName,optimalSolutionLength,tolerance):
        #e for elitist, a for Ant Colony System
        self.algorithm = algorithm
        #number of ants in the colony
        self.numAnts = numAnts
        #number of search iterations
        self.numIter = numIter
        #the name of the file for the problem
        self.problemName = problemName
        #A list of three tuples of index, xcord, ycord
        self.problem = []
        #the length of the known best solution
        self.optimalSolutionLength = optimalSolutionLength
        #the tolerance for how close our solution is to the best
        self.tolerance = tolerance

    def readProblem(self):
        file = open(self.problemName, "r+")
        inp = file.readlines()
        #cleans the lines
        inp = [line.strip("\n") for line in inp]
        for r in inp:
            cleanRow = []
            row = r.split(" ")
            for num in row:
                #this catches lines that are not coordinates
                try:
                    cleanRow += [float(num)]
                except:
                    continue
            #convert first index to int, since it is not a coordinate
            try:
                cleanRow[0] = int(cleanRow[0])
            except:
                continue
            #some problems have rows with numbers that aren't coordinates
            #makes sure each "row " is the correct length
            if len(cleanRow) > 2:
                self.problem += [cleanRow]

    def run(self):
        self.readProblem()
        #initialize ACO
        aco = ACO(self.algorithm,self.numAnts,self.numIter,self.problem,self.optimalSolutionLength,self.tolerance)
        #start timer
        start_time = time.time()
        #run solve
        solution,solutionCost = aco.solve()
        #format the solution returned so it starts with 1 index
        formatedSol = self.formatSolution(solution)
        #print results
        print("solution: ", formatedSol)
        print()
        print("solutionCost: ", solutionCost)
        print("Total Time: {} seconds.".format(time.time() - start_time))

    def formatSolution(self,solution):
        #just get node number from nodes in solution
        fSolution = [i[0] for i in solution]
        #look for the index of node 1
        indexOfFirstNode = fSolution.index(1)
        #order the solytion so node 1 comes first
        orderedSolution = fSolution[indexOfFirstNode:len(fSolution)] + fSolution[0:indexOfFirstNode]
        return orderedSolution


#Get Terminal Input
algorithm = sys.argv[1]
numAnts = sys.argv[2]
numIter = sys.argv[3]
problemName = sys.argv[4]
optimalSolutionLength = sys.argv[5]
tolerance = sys.argv[6]

optimize = Optimize(algorithm,numAnts,numIter,problemName,optimalSolutionLength,tolerance)
optimize.run()
