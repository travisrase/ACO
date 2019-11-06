import sys
from ACO import ACO
class Optimize:
    def __init__(self,algorithm,numAnts,numIter,alpha,beta,rho,elitismFactor,epsilon,tao0,q0,problemName):
        #g for greedy, a for Ant Colony System
        self.algorithm = algorithm
        #number of ants in the colony
        self.numAnts = numAnts
        #number of search iterations
        self.numIter = numIter
        #the degree of influence of the phermones
        self.aplha = alpha
        #the degree of influnece of the hueristic component
        self.beta = beta
        #phermone evaporation factor
        self.rho = rho
        #elitism factor in Elitist Ant System
        self.elitismFactor = elitismFactor
        self.epsilon = epsilon
        self.tao0 = tao0
        #probability that the ant will choose the best leg for the next leg of the tour
        self.q0 = q0
        #the name of the file for the problem
        self.problemName = problemName
        #A list of three tuples of index, xcord, ycord
        self.problem = []


    def readProblem(self):
        file = open(self.fileName, "r+")
        inp = file.readlines()
        inp = [line.strip("\n") for line in inp]
        for r in inp:
            cleanRow = []
            row = r.split(" ")
            for num in row:
                try:
                    cleanRow += [int(num)]
                except:
                    continue
            self.problem += [cleanRow]

    def run(self):
        self.readProblem()
        if self.algorithm == "g":
            print()
        else:
            aco = ACO(self.numAnts,self.numIter,self.aplha,self.beta,self.rho,self.elitismFactor,self.epsilon,self.tao0,self.q0,self.problem)
            solution = aco.solve()


#Get Terminal Input
algorithm = sys.argv[1]
numAnts = sys.argv[2]
numIter = sys.argv[3]
aplha = sys.argv[4]
beta = sys.argv[5]
rho = sys.argv[6]
elitismFactor = sys.argv[7]
epsilon = sys.argv[8]
tao0 = sys.argv[9]
q0 = sys.argv[10]
problemName = sys.argv[11]

optimize = Optimize(algorithm,numAnts,numIter,aplha,beta,rho,elitismFactor,epsilon,tao0,q0,problemName)
