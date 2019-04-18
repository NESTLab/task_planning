'''
Centralized Heuristic Algorithm for Team Orienteering Problem with Fuel Constrained Robots

This script requires that
    *'numpy'
be installed within the Python environment you are running this script in.

This file can also be imported as a module and contains the following functions:
    * ILS - finds a feasible path given certain constraints
'''


# Importing existing python modules
import numpy as np
import copy
import pprint
import random as rnd
import math
from itertools import cycle
import sys

# Importing required project modules
import environment as env
from visualization import Visualization_TOPF


class Heuristics_TOPF:
    def __init__(self, K, T, D, S, T_loc, D_loc, N_loc, L, T_max, velocity, seed):
        self.K = K
        self.T = T
        self.D = D
        self.S = S
        self.T_loc = T_loc
        self.D_loc = D_loc
        self.N_loc = N_loc
        self.L = L
        self.T_max = T_max
        self.vel = velocity
        self.thisSeed = seed

        # Total nodes
        self.N = T + D
        # Defining edges of the graph
        self.edges = [(i, j) for i in self.N for j in self.N if i != j]

        # Distance between nodes = edge weight
        self.c = {t: np.linalg.norm(np.array(N_loc.get(t[0])) - np.array(N_loc.get(t[1]))) for t in iter(self.edges)}
        self.f = self.c


        conversionToBeReversed = False

        self.maxIterations = 25

        # Final heuristic Solution to be stored here
        self.arcsInOrderHeuristic = {k: [(self.S[0], self.S[0])] for k in self.K}

        # ILS related parameters
        #self.noImprovementIterations = 0
        #self.diversificationParam = 0





    def fuelCheck(self, path, maxFuel):
        if not all(isinstance(n, tuple) for n in path):
            path = self.pathArcRepresentation(path)

        fuelConsumed = 0
        fuelCheckPassed = False
        for arc in path:
            if arc[0] not in self.D or arc[1] not in self.D or arc[0] != arc[1]:
                fuelForArc = self.f[arc]
            elif arc[0] in self.D and arc[1] in self.D and arc[0] == arc[1]:
                fuelForArc = 0
            else:
                #print("Curr arc under process (%s, %s)" % (arc[0], arc[1]))
                raise ValueError("fuelCheck() is behaving strangely for this arc.")
            if fuelConsumed + fuelForArc <= maxFuel:
                fuelConsumed += fuelForArc
            else:
                return fuelCheckPassed, fuelConsumed
            if arc[1] in self.D:
                fuelConsumed = 0
        fuelCheckPassed = True
        return fuelCheckPassed, fuelConsumed



    def timeCheck(self, path, maxTime):
        if not all(isinstance(n, tuple) for n in path):
            path = self.pathArcRepresentation(path)

        timeSpent = 0
        timeCheckPassed = False
        for arc in path:
            if arc[0] not in self.D or arc[1] not in self.D or arc[0] != arc[1]:
                timeForArc = self.c[arc]
            elif arc[0] in self.D and arc[1] in self.D and arc[0] == arc[1]:
                timeForArc = 0
            else:
                #print("Curr arc under process (%s, %s)" % (arc[0], arc[1]))
                raise ValueError("fuelCheck() is behaving strangely for this arc.")
            if timeSpent + timeForArc <= maxTime:
                timeSpent += timeForArc
            else:
                return timeCheckPassed, timeSpent
        timeCheckPassed = True
        return timeCheckPassed, timeSpent


    def predecessorSuccessorComponents(self, path, arc):
        arc_loc = path.index(arc)
        predecessor = path[:arc_loc]
        successor = path[arc_loc + 1:]
        if len(path) <= 1:
            predecessor = []
            successor = []
        elif arc_loc == len(path) - 1:
            successor = []
        # print('predecessor' + str(predecessor))
        # print('successor '+ str(successor))
        return predecessor, successor


    def checkadditionsInPath(self, path, t):
        conversionToBeReversed = False
        if not all(isinstance(n, tuple) for n in path):
            path = self.pathArcRepresentation(path)
            conversionToBeReversed = True
        for arc in path:
            predecessor, successor = self.predecessorSuccessorComponents(path, arc)
            newPath = predecessor + [(arc[0], t)] + [(t, arc[1])] + successor
            # print(path)
            # print(newPath)
            fuelCheckPassed, _ = self.fuelCheck(newPath, self.L)
            timeCheckPassed, _ = self.timeCheck(newPath, self.T_max)
            if fuelCheckPassed and timeCheckPassed:
                if conversionToBeReversed:
                    newPath = self.pathNodeRepresentation(newPath)
                return newPath, True
        if conversionToBeReversed:
            path = self.pathNodeRepresentation(path)
        return path, False



    def computeGreedySolution(self):
        # Algorithm 1
        # -----------
        # For each robot
        fuelConsumedHeuristic = {k: 0 for k in self.K}
        T_Heuristic = self.T.copy()
        tasksAssigned = []
        for k in self.K:
            for t in T_Heuristic:
                newPath, bool_c = self.checkadditionsInPath(self.arcsInOrderHeuristic[k], t)
                if bool_c:
                    self.arcsInOrderHeuristic[k] = newPath
                    fuelConsumedHeuristic[k] = self.pathLength(newPath)
                    #pprint.pprint(self.arcsInOrderHeuristic)
                    #pprint.pprint(fuelConsumedHeuristic)
                    tasksAssigned.append(t)
                    T_Heuristic.remove(t)
                    #print(T_Heuristic)
        # Algorithm 4
        # -----------
        # Add Depots in paths
        D_Heuristic = self.D[:]
        D_Heuristic.remove(self.S[0])
        for k in self.K:
            for d in D_Heuristic:
                newPath, bool_c = self.checkadditionsInPath(self.arcsInOrderHeuristic[k], d)
                if bool_c:
                    self.arcsInOrderHeuristic[k] = newPath
                    D_Heuristic.remove(d)
        #pprint.pprint(self.arcsInOrderHeuristic)


    def pathNodeRepresentation(self, arcBasedPath):
        nodeBasedPath = [arc[0] for arc in arcBasedPath]
        nodeBasedPath.append(arcBasedPath[-1][1])
        # print (nodeBasedPath)
        return nodeBasedPath

    # Similarly, we need a function to convert from a node based
    # representation to an arc based representation
    def pathArcRepresentation(self, nodeBasedPath):
        it1 = iter(nodeBasedPath)
        it2 = iter(nodeBasedPath)
        it2.__next__()
        arcBasedPath = [(start, end) for start, end in zip(it1, it2)]
        # print (arcBasedPath)
        return arcBasedPath



    # We need a function that returns the XY Coordinates, given a node name
    def xyCoordinates(self, nodeName):
        if nodeName in self.T:
            return self.T[nodeName][0], self.T[nodeName][1]
        elif nodeName in self.D:
            return self.D[nodeName][0], self.D[nodeName][1]
        else:
            return "No node with this name found!"

    def pathLength(self, path):
        l = 0
        if not all(isinstance(n, tuple) for n in path):
            path = self.pathArcRepresentation(path)
        for arc in path:
            if arc[0] not in self.D or arc[1] not in self.D or arc[0] != arc[1]:
                l += self.c[arc]
            elif arc[0] in self.D and arc[1] in self.D and arc[0] == arc[1]:
                l += 0
            else:
                #print("Curr arc under process (%s, %s)" % (arc[0], arc[1]))
                raise ValueError(
                    "pathLength() is behaving strangely for this arc.")
        return l

    # Lets define our own tour cost function
    def tourCost(self, tour):
        costOfAllTours = 0
        # costOfEachTour = {k:pathLength(tour[k]) for k in K}
        for k in self.K:
            costOfAllTours += self.pathLength(tour[k])
        return costOfAllTours



    # Stochastic 2-opt
    def stochasticTwoOpt(self, perm, maxIter):
        _, fuelConsumedBefore2Opt = self.fuelCheck(perm, self.L)
        _, timeSpentBefore2Opt = self.timeCheck(perm, self.T_max)
        if len(perm) <= 3:
            #print('No 2opt move possible with only %d nodes' % len(perm))
            return perm
        while maxIter > 0:

            result = perm[:]  # make a copy, but start and end nodes may not be
            # considered
            size = len(result)
            # Ensure that start/end nodes are not selected
            exclude = set([0])
            exclude.add(size - 1)
            # select indices of two random points in the tour
            p1, p2 = rnd.randrange(0, size), rnd.randrange(0, size)
            while p1 in exclude:
                p1 = rnd.randrange(0, size)
            # do this so as not to overshoot tour boundaries
            exclude.add(p1)
            if p1 == 0:
                exclude.add(size - 1)
            else:
                exclude.add(p1 - 1)

            if p1 == size - 1:
                exclude.add(0)
            else:
                exclude.add(p1 + 1)
            if len(exclude) == len(perm):
                # exclude has elements equal to total number of elements. No 2opt
                # possible
                maxIter -= 1
                continue

            while p2 in exclude:
                p2 = rnd.randrange(0, size)

            # to ensure we always have p1<p2
            if p2 < p1:
                p1, p2 = p2, p1

            # now reverse the tour segment between p1 and p2
            result[p1:p2] = reversed(result[p1:p2])
            #print("Iteration: " + str(maxIter))
            #print("2-Opt move attempted with edge (%s, %s) and (%s, %s)"
                  # % (perm[p1], perm[p1 + 1], perm[p2 - 1], perm[p2]))
            #print("New attempted path is :" + str(result))

            # Check if the path is fuel and time feasible
            fuelCheckPassed, fuelConsumedAfter2Opt = self.fuelCheck(result, self.L)
            timeCheckPassed, timeSpentAfter2Opt = self.timeCheck(result, self.T_max)

            if timeCheckPassed:
                # Check if the current time and fuel is less than previous ones
                if timeSpentAfter2Opt < timeSpentBefore2Opt:
                    # This is a valid 2opt
                    #print("2-Opt move successful with edge (%s, %s) and (%s, %s)"
                          # % (perm[p1], perm[p1 + 1], perm[p2 - 1], perm[p2]))
                    #print("Previous Path: (Time Cost: %.2f)" %
                          # timeSpentBefore2Opt)
                    #pprint.pprint(perm)
                    #print("New Path: (Time Cost: %.2f)" %
                          # timeSpentAfter2Opt)
                    #pprint.pprint(result)
                    return result

            maxIter -= 1
        #print("No 2-Opt move feasible")
        #pprint.pprint(perm)
        return perm



    # Local Seach Step
    def localSearch(self, best, maxIter):
        while maxIter > 0:
            candidate = {}
            candidate["permutation"] = self.stochasticTwoOpt(best["permutation"])
            candidate["cost"] = self.tourCost(candidate["permutation"])
            if candidate["cost"] < best["cost"]:
                best = candidate

            maxIter -= 1

        return best



    def insert(self, path, nonVisitedTasks):
        newPath = path[:]
        # Try to insert all the depots in the path that are not there
        for d in self.D:
            if d not in newPath:
                newPath, _ = self.checkadditionsInPath(newPath, d)
        # Try to insert all the nonVisitedTasks
        for t in nonVisitedTasks:
            newPath, _ = self.checkadditionsInPath(newPath, t)
        return newPath



    def nodesNotInTour(self, tour):
        nodesInTour = []
        for k in self.K:
            for n in self.N:
                if n in tour[k]:
                    nodesInTour.append(n)
        nodesNotInTour = [i for i in self.N if i not in nodesInTour]
        return nodesNotInTour

    def tasksNotInTour(self, tour):
        tasksInTour = []
        for k in self.K:
            for t in self.T:
                if t in tour[k]:
                    tasksInTour.append(t)
        tasksNotInTour = [i for i in self.T if i not in tasksInTour]
        return tasksNotInTour



    def pickTwoRandomPaths(self, tour):
        keys = rnd.sample(list(tour), 2)
        return keys[0], keys[1]



    def pickRandomNode(self, path):
        path = path[1:-1]
        index = rnd.randint(0, len(path) - 1)
        rndNode = path[index]
        index = index + 1  # Because we removed the first element from original path
        return rndNode, index



    def swap(self, tour, maxIter):
        while maxIter > 0:
            prevTour = copy.deepcopy(tour)
            # Swap tours from one path to the other path
            aK, bK = self.pickTwoRandomPaths(tour)
            _, fuelConsumedBeforeSwap1 = self.fuelCheck(tour[aK], self.L)
            _, fuelConsumedBeforeSwap2 = self.fuelCheck(tour[bK], self.L)
            _, timeSpentBeforeSwap1 = self.timeCheck(tour[aK], self.T_max)
            _, timeSpentBeforeSwap2 = self.timeCheck(tour[bK], self.T_max)
            fuelConsumedBeforeSwap = fuelConsumedBeforeSwap1 + fuelConsumedBeforeSwap2
            timeSpentBeforeSwap = timeSpentBeforeSwap1 + timeSpentBeforeSwap2
            #print("Iteration: " + str(maxIter))
            # Pick two nodes randomly from each path, other than start nodes
            if timeSpentBeforeSwap1 > 0:
                node1, n1Idx = self.pickRandomNode(tour[aK])
            else:
                #print("No Swap possible between paths of robots %s and %s." % (aK, bK))
                maxIter -= 1
                continue
            if timeSpentBeforeSwap2 > 0:
                node2, n2Idx = self.pickRandomNode(tour[bK])
            else:
                #print("No Swap possible between paths of robots %s and %s." % (aK, bK))
                maxIter -= 1
                continue

            # Try swapping them in their paths
            newaK = copy.deepcopy(tour[aK])
            newbK = copy.deepcopy(tour[bK])
            newaK[n1Idx], newbK[n2Idx] = newbK[n2Idx], newaK[n1Idx]
            # print("Swap Attempted Between %s of path for robot %s and %s of path for robot %s "
            #                            % (node1, aK, node2, bK))

            # Check if the paths are fuel and time feasible
            fuelCheckPassed1, fuelConsumed1 = self.fuelCheck(newaK, self.L)
            fuelCheckPassed2, fuelConsumed2 = self.fuelCheck(newbK, self.L)
            fuelConsumedAfterSwap = fuelConsumed1 + fuelConsumed2

            timeCheckPassed1, timeSpent1 = self.timeCheck(newaK, self.T_max)
            timeCheckPassed2, timeSpent2 = self.timeCheck(newbK, self.T_max)
            timeSpentAfterSwap = timeSpent1 + timeSpent2

            if fuelCheckPassed1 and fuelCheckPassed2:
                if timeCheckPassed1 and timeCheckPassed2:
                    # Check if the current time and fuel is less than previous ones
                    if timeSpentAfterSwap < timeSpentBeforeSwap:
                        # This is a valid swap
                        # print("Swap Done Between %s of path for robot %s and %s of path for robot %s "
                        #                  % (node1, aK, node2, bK))
                        #print("Previous Tour: (Time Cost: %.2f)" %
                              #timeSpentBeforeSwap)
                        # pprint.pprint(prevTour)
                        tour[aK] = newaK[:]
                        tour[bK] = newbK[:]
                        #print("New Tour: (Time Cost: %.2f)" %
                              #timeSpentAfterSwap)
                        # pprint.pprint(tour)
                        return tour
            maxIter -= 1
        #print("No swap executed:")
        #print("Previous Tour:")
        #pprint.pprint(prevTour)
        #print("NewTour:")
        #pprint.pprint(tour)
        return tour



    def move(self, tour):
        modifiedTour = copy.deepcopy(tour)
        # Arrange the tour paths in increasing length of paths
        sortedPathsByLength = sorted(tour.items(), key=lambda kv: len(kv[1]))
        sortedPathsByLength = [i for i, j in sortedPathsByLength]
        unexplored_paths = sortedPathsByLength[:]
        totalTimeBeforeMove = 0
        totalFuelConsumedBeforeMove = 0
        for k in self.K:
            _, temp = self.fuelCheck(tour[k], self.L)
            totalFuelConsumedBeforeMove += temp
            _, temp = self.timeCheck(tour[k], self.T_max)
            totalTimeBeforeMove += temp
        for k in sortedPathsByLength:
            unexplored_paths.remove(k)
            for node in tour[k][1:-1]:  # Skip the start and end nodes
                if node in self.N:
                    # Check if this node can be added to some other path by
                    # checking fuel and time checks
                    for k2 in unexplored_paths:
                        tempTour = copy.deepcopy(tour)
                        tempTour[k2], bool_c = self.checkadditionsInPath(
                            tour[k2], node)
                        if bool_c:
                            tempTour[k].remove(node)
                            totalFuelConsumedAfterMove = 0
                            totalTimeAfterMove = 0
                            for k3 in self.K:
                                _, temp = self.fuelCheck(tempTour[k3], self.L)
                                totalFuelConsumedAfterMove += temp
                                _, temp = self.timeCheck(tempTour[k3], self.T_max)
                                totalTimeAfterMove += temp
                            if totalTimeAfterMove < totalTimeBeforeMove:
                                modifiedTour = copy.deepcopy(tempTour)
                                # print("Node %s of path for robot %s moved to path of robot %s "
                                #        get_ipython().run_line_magic('(node,', 'k, k2))')
                                # print("Previous Tour: (Time Cost: %.2f)" %
                                #       totalTimeBeforeMove)
                                #pprint.pprint(tour)
                                # print("New Tour: (Time Cost: %.2f)" %
                                #       totalTimeAfterMove)
                                #pprint.pprint(modifiedTour)
                                break
        return modifiedTour




    # Attempt to remove extra depots
    def removeExtraDepots(self, tour):
        modfiedTour = copy.deepcopy(tour)
        for k in self.K:
            for node in tour[k]:
                tempPath = modfiedTour[k][:]
                if node in self.D and node != self.S[0]:
                    tempPath.remove(node)
                    fuelCheckPassed, _ = self.fuelCheck(tempPath, self.L)
                    timeCheckPassed, _ = self.timeCheck(tempPath, self.T_max)
                    if fuelCheckPassed and timeCheckPassed:
                        modfiedTour[k].remove(node)
                        #print('Depot %s removed from path of robot %s' % (node, k))
        return modfiedTour



    # Perturbation
    def perturb(self, tour, A):
        modifiedTour = copy.deepcopy(tour)
        # compute no of depots and no of tasks in tour
        noOfDepotsInCurrSol = 1
        noOfTasksInCurrSol = 0
        for k in self.K:
            for node in tour[k]:
                if node in self.D and node != self.S[0]:
                    noOfDepotsInCurrSol += 1
                if node in self.T:
                    noOfTasksInCurrSol += 1

        rho = noOfDepotsInCurrSol / (noOfTasksInCurrSol + noOfDepotsInCurrSol)
        # Remove rho*A percent depots randomly from the current path
        # Remove (1-rho)*A percent tasks randomly from the current path
        for k in self.K:
            # Calculate indicies of depots and tasks in curr path
            depotIndiciesInCurrPath = []
            taskIndiciesInCurrPath = []
            for idx, node in enumerate(tour[k]):
                if node in self.D and node != self.S[0]:
                    depotIndiciesInCurrPath.append(idx)
                if node in self.T:
                    taskIndiciesInCurrPath.append(idx)
            # Now, remove rho*A percent depots from current path
            noOfDepotsToBeRemoved = math.ceil(0.3 * rho * A * len(depotIndiciesInCurrPath))
            #print("%d depot(s) to be removed from path %s" % (noOfDepotsToBeRemoved, k))
            depotsInPath = [
                n for n in modifiedTour[k] if n in self.D and n != self.S[0]]
            if depotsInPath:
                for i in range(noOfDepotsToBeRemoved):
                    randomDepotIdx = rnd.choice(depotIndiciesInCurrPath)
                    randomDepot = tour[k][randomDepotIdx]
                    modifiedTour[k].remove(randomDepot)
                    depotIndiciesInCurrPath.remove(randomDepotIdx)
                    # print("Depot %s removed from path of %s"
                    #       % (randomDepot, k))
            else:
                #print("No Depot in path %s to remove." % k)
                continue
            # Remove (1-rho)*A percent tasks from the current path
            noOfTasksToBeRemoved = math.ceil(0.3 * (1 - rho) * A * len(taskIndiciesInCurrPath))
            #print("%d tasks to be removed from path %s" % (noOfTasksToBeRemoved, k))
            tasksInPath = [
                n for n in modifiedTour[k] if n in self.T and n != self.S[0]]
            if tasksInPath:
                for i in range(noOfTasksToBeRemoved):
                    randomTaskIdx = rnd.choice(taskIndiciesInCurrPath)
                    randomTask = tour[k][randomTaskIdx]
                    modifiedTour[k].remove(randomTask)
                    taskIndiciesInCurrPath.remove(randomTaskIdx)
                    # print("Task %s removed from path of %s"
                    #       % (randomTask, k))
            else:
                #print("No Task in path %s to remove." % k)
                continue

        return modifiedTour


    def taskCount(self, tour):
        taskCountInTour = 0
        for k in self.K:
            for node in tour[k]:
                if node in self.T:
                    taskCountInTour += 1
        return taskCountInTour



    # Lets make a function that compares current solutoin with best solution
    def compareSolWithBest(self, currSol, bestSol):
        currSolTourCost = self.tourCost(currSol)
        bestSolTourCost = self.tourCost(bestSol)
        currSolTaskCount = self.taskCount(currSol)
        bestSolTaskCount = self.taskCount(bestSol)
        gamma = 0.0001
        currSolQuality = currSolTaskCount - gamma * currSolTourCost
        bestSolQuality = bestSolTaskCount - gamma * bestSolTourCost
        if currSolQuality > bestSolQuality:
            return currSol, True
        return bestSol, False



    # Main loop for Iterated Local Search

    def ILS(self):
        # thisSeed = rnd.randrange(sys.maxsize)
        rnd.seed(self.thisSeed)
        print("Random Seed:", self.thisSeed)

        # Dict for holding the cnadidate solution
        candidateSolution = {}


        # Get the greedy solution computed
        self.computeGreedySolution()

        noImprovementIterations = 0
        diversificationParam = 0

        for k in self.K:
            candidateSolution[k] = self.pathNodeRepresentation(self.arcsInOrderHeuristic[k])
            # Lets not initialize it at all
            # candidateSolution[k] = [S[0], S[0]]
        # Initial Solution -------------------------------------------------------
        bestSolution = copy.deepcopy(candidateSolution)

        # Local Search  ----------------------------------------------------------
        # Insert
        self.nonVisitedTasks = self.tasksNotInTour(candidateSolution)
        for k in self.K:
            candidateSolution[k] = self.insert(candidateSolution[k], self.nonVisitedTasks)
            self.nonVisitedTasks = self.tasksNotInTour(candidateSolution)
        bestSolution, _ = self.compareSolWithBest(candidateSolution, bestSolution)
        # Swap
        candidateSolution = self.swap(candidateSolution, self.maxIterations)
        bestSolution, _ = self.compareSolWithBest(candidateSolution, bestSolution)
        # Move
        candidateSolution = self.move(candidateSolution)
        bestSolution, _ = self.compareSolWithBest(candidateSolution, bestSolution)
        # 2-opt
        for k in self.K:
            candidateSolution[k] = self.stochasticTwoOpt(candidateSolution[k], self.maxIterations)
        bestSolution, _ = self.compareSolWithBest(candidateSolution, bestSolution)
        # Remove Depots
        candidateSolution = self.removeExtraDepots(candidateSolution)
        bestSolution, _ = self.compareSolWithBest(candidateSolution, bestSolution)
        # 2-opt
        #    for k in K:
        #        candidateSolution[k] = stochasticTwoOpt(
        #            candidateSolution[k], maxIterations)
        #    bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)
        # Move
        candidateSolution = self.move(candidateSolution)
        bestSolution, _ = self.compareSolWithBest(candidateSolution, bestSolution)

        print("Local search ended")
        # Local Search Ended -----------------------------------------------------

        while noImprovementIterations < 10:
            #print("No Improvement Itertion %d." % noImprovementIterations)
            #print("Random Seed:", thisSeed)

            # Perturb
            candidateSolution = self.perturb(candidateSolution, diversificationParam)
            # Local Search  ----------------------------------------------------------
            # Insert
            self.nonVisitedTasks = self.tasksNotInTour(candidateSolution)
            for k in self.K:
                candidateSolution[k] = self.insert(
                    candidateSolution[k], self.nonVisitedTasks)
                self.nonVisitedTasks = self.tasksNotInTour(candidateSolution)
            bestSolution, boolImproved = self.compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0
            # Swap
            candidateSolution = self.swap(candidateSolution, self.maxIterations)
            bestSolution, boolImproved = self.compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0
            # Move
            candidateSolution = self.move(candidateSolution)
            bestSolution, boolImproved = self.compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0
            # 2-opt
            for k in self.K:
                candidateSolution[k] = self.stochasticTwoOpt(candidateSolution[k], self.maxIterations)
            bestSolution, boolImproved = self.compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0

            # Remove Depots
            candidateSolution = self.removeExtraDepots(candidateSolution)
            bestSolution, boolImproved = self.compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0
            #        # 2-opt
            #        for k in K:
            #            candidateSolution[k] = stochasticTwoOpt(
            #                                    candidateSolution[k], maxIterations)
            #        bestSolution, boolImproved = compareSolWithBest(
            #                                    candidateSolution, bestSolution)
            #        if boolImproved:
            #            noImprovementIterations = 0
            # Move
            candidateSolution = self.move(candidateSolution)
            bestSolution, boolImproved = self.compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0

            noImprovementIterations += 1

            # Local Search Ended -----------------------------------------------------
            diversificationParam += 1
            # Check noOfnodes in the smallest path. We cannot remove more than those nodes
            smallestNoOfNodes = 1e6
            for k in self.K:
                noOfNodesInThisPath = len(candidateSolution[k])
                if noOfNodesInThisPath < smallestNoOfNodes:
                    smallestNoOfNodes = noOfNodesInThisPath
            if diversificationParam > smallestNoOfNodes:
                diversificationParam = diversificationParam - smallestNoOfNodes

        #print('Best Solution:')
        #pprint.pprint(bestSolution)
        cost = self.tourCost(bestSolution)
        taskcount = self.taskCount(bestSolution)
        gamma = 0.0001
        quality = taskcount - gamma * cost

        return bestSolution, self.c, self.thisSeed, quality





def main():

    '''Run Heuristics with randomly generated input'''
    noOfRobots = 3
    noOfTasks = 8
    noOfDepots = 2
    L = 200
    T_max = 500
    velocity = 1
    # randomly generated locations of tasks and robots
    K, T, D, S, T_loc, D_loc, N_loc = env.generate_test_instance_topf(noOfRobots, noOfTasks, noOfDepots)

    inst = Heuristics_TOPF(K, T, D, S, T_loc, D_loc, N_loc, L, T_max, velocity)
    finalHeuristicSolution, c, seed, cost = inst.ILS()


    arcsInOrderHeuristic = {}
    for k in K:
        arcsInOrderHeuristic[k] = inst.pathArcRepresentation(finalHeuristicSolution[k])
    print(arcsInOrderHeuristic)

    # Plot the routes using plotly interactive GUI
    draw = Visualization_TOPF(K, T, D, S, T_loc, D_loc, c)
    # filename if the plot to be saved
    name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots) + '.html'
    # plot and save
    auto_open_flag = 1
    draw.save_plot_topf_heuristic(arcsInOrderHeuristic, name, auto_open_flag)


if __name__ == "__main__":
    main()
