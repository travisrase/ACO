# ACO
Ant Colony Optimization Algorithm

This program can be used to find solutions to traveling salesman problems. The TSP problems must be given as a test file list 
of x,y coordinate locations on a euclidian grid. The program has an Ant Colont System and an Elitist Ant System Implemented.

The program also has a folder of test problems that the alogrithm can be run on. The PDF file tsp-optimal-tour-lengths.pdf has
a list of all the problems in the folder, their size, and their optimal solution. 

To run the program you must run the following command

$ Python3 Optimize.py (a or e) (numAnts) (numIterations) problems/problemName (optimal solution length) (tolerance)

(a or e): a = ACS algorithm, e = Elitist Ant System
(numAnts): The number of ants in the ant system
problemName: the name of the file in problems
(otpimal solution length): The cost of the optimal tour. Can be found in tsp-optimal-tour-lengths.pdf
(tolerance): The tolerance for a given solution. A 1 means the algorithm will only terminate when an optimal solution is found. 
A .8 would mean the algorithm would terminate if it found a solution 80% as good as the optimal. 
