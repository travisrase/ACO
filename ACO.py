import numpy as np
import random
class ACO:
    def __init__(self,numAnts,numIter,alpha,beta,rho,epsilon,tao0,q0,problem):
        #number of ants in the colony
        self.numAnts = numAnts
        #number of search iterations
        self.numIter = numIter
        #the degree of influence of the phermones
        self.aplha = alpha
        self.beta = beta
        self.rho = rho
        self.epsilon = epsilon
        self.tao0 = tao0
        self.q0 = q0
        self.problem = []
        #matrix keeping track of phermones between two nodes
        self.permoneMatrix = []
        #a list of all ant solutions
        self.ants = []

    def initPhermoneMatrix(self):
        #init n x n 2d array where n = size of the problem
        self.permoneMatrix = np.zeros[(len(self.problem),(len(self.problem))]

    def solve(self):
        #initialize the phermone matrix
        self.initPhermoneMatrix()
        i = 0
        while (i < self.numIter):
            for ant in range(self.numAnts):
                #the path created by a given ant
                path = []
                #the nodes that have not yet been visited by the ant
                unvisitedNodes = self.problem
                for node in range(len(self.problem)):
                    path,unvisitedNodes = self.getNextNode(path,unvisitedNodes)


    def getNextNode(self,path,unvisitedNodes):
        nextNodeIndex
        if node == 0:
            #randomly generate starting node index
            startIndex = random.randrange(0,len(self.problem))
            startNode = remove(startIndex)
        else:
        print()
