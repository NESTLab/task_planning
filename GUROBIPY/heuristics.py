'''
Centralized Heuristic Algorithm for Team Orienteering Problem with Fuel Constrained Robots

This script requires that
    *'numpy'
be installed within the Python environment you are running this script in.

This file can also be imported as a module and contains the following functions:
    * planner - finds a feasible path given certain constraints
'''


# Importing existing python modules
import numpy as np

# Importing required project modules
import environment as env
from visualization import Visualization_TOPF


class Greedy:
    def __init__(self):
        self.data = []

    def fuelCheck(path, maxFuel):
        conversionToBeReversed = False
        if not all(isinstance(n, tuple) for n in path):
            path = pathArcRepresentation(path)

        fuelConsumed = 0
        fuelCheckPassed = False
        for arc in path:
            if arc[0] not in D or arc[1] not in D or arc[0] != arc[1]:
                fuelForArc = f[arc]
            elif arc[0] in D and arc[1] in D and arc[0] == arc[1]:
                fuelForArc = 0
            else:
                print("Curr arc under process (%s, %s)" % (arc[0], arc[1]))
                raise ValueError("fuelCheck() is behaving strangely for this arc.")
            if fuelConsumed + fuelForArc <= maxFuel:
                fuelConsumed += fuelForArc
            else:
                return fuelCheckPassed, fuelConsumed
            if arc[1] in D:
                fuelConsumed = 0
        fuelCheckPassed = True
        return fuelCheckPassed, fuelConsumed

    # In[43]:

    def timeCheck(path, maxTime):
        if not all(isinstance(n, tuple) for n in path):
            path = pathArcRepresentation(path)

        timeSpent = 0
        timeCheckPassed = False
        for arc in path:
            if arc[0] not in D or arc[1] not in D or arc[0] != arc[1]:
                timeForArc = c[arc]
            elif arc[0] in D and arc[1] in D and arc[0] == arc[1]:
                timeForArc = 0
            else:
                print("Curr arc under process (%s, %s)" % (arc[0], arc[1]))
                raise ValueError("fuelCheck() is behaving strangely for this arc.")
            if timeSpent + timeForArc <= maxTime:
                timeSpent += timeForArc
            else:
                return timeCheckPassed, timeSpent
        timeCheckPassed = True
        return timeCheckPassed, timeSpent

    # In[44]:

    def predecessorSuccessorComponents(path, arc):
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

    # In[45]:

    def checkadditionsInPath(path, t):
        conversionToBeReversed = False
        if not all(isinstance(n, tuple) for n in path):
            path = pathArcRepresentation(path)
            conversionToBeReversed = True
        for arc in path:
            predecessor, successor = predecessorSuccessorComponents(path, arc)
            newPath = predecessor + [(arc[0], t)] + [(t, arc[1])] + successor
            # print(path)
            # print(newPath)
            fuelCheckPassed, _ = fuelCheck(newPath, L)
            timeCheckPassed, _ = timeCheck(newPath, T_max)
            if fuelCheckPassed and timeCheckPassed:
                if conversionToBeReversed:
                    newPath = pathNodeRepresentation(newPath)
                return newPath, True
        if conversionToBeReversed:
            path = pathNodeRepresentation(path)
        return path, False

    # In[46]:

    # Algorithm 1
    # -----------
    # For each robot
    fuelConsumedHeuristic = {k: 0 for k in K}
    arcsInOrderHeuristic = {k: [(S[0], S[0])] for k in K}
    print(arcsInOrderHeuristic)
    T_Heuristic = T.copy()
    tasksAssigned = []
    for k in K:
        for t in T_Heuristic:
            newPath, bool_c = checkadditionsInPath(arcsInOrderHeuristic[k], t)
            if bool_c:
                arcsInOrderHeuristic[k] = newPath
                fuelConsumedHeuristic[k] = pathLength(newPath)
                pprint.pprint(arcsInOrderHeuristic)
                pprint.pprint(fuelConsumedHeuristic)
                tasksAssigned.append(t)
                T_Heuristic.remove(t)
                print(T_Heuristic)

    # Algorithm 4
    # -----------
    # Add Depots in paths
    D_Heuristic = D[:]
    D_Heuristic.remove(S[0])
    for k in K:
        for d in D_Heuristic:
            newPath, bool_c = checkadditionsInPath(arcsInOrderHeuristic[k], d)
            if bool_c:
                arcsInOrderHeuristic[k] = newPath
                D_Heuristic.remove(d)

    pprint.pprint(arcsInOrderHeuristic)

    #  Create $\texttt{arcsInOrderHeuristic}$ and $\texttt{remainingFuelHeuristic}$

    # In[47]:

    remainingFuel = {t: 0 for t in T}

    #  Plot the above Heuristic Solution

    # In[48]:

    pprint.pprint(arcsInOrderHeuristic)
    fig = drawArena(T_loc, D_loc, arcsInOrderHeuristic, remainingFuel, 1)
    py.iplot(fig)

    # py.plot(fig, filename='temp2.html')

    # In[49]:

    # , filename='temp1.html' Now, lets start working on iterated local search
    # First, we need a function that converts a path in terms of arcs to
    # a path in terms of nodes
    def pathNodeRepresentation(arcBasedPath):
        nodeBasedPath = [arc[0] for arc in arcBasedPath]
        nodeBasedPath.append(arcBasedPath[-1][1])
        # print (nodeBasedPath)
        return nodeBasedPath

    # Similarly, we need a function to convert from a node based
    # representation to an arc based representation
    def pathArcRepresentation(nodeBasedPath):
        it1 = iter(nodeBasedPath)
        it2 = iter(nodeBasedPath)
        it2.__next__()
        arcBasedPath = [(start, end) for start, end in zip(it1, it2)]
        # print (arcBasedPath)
        return arcBasedPath

    # In[50]:

    # We need a function that returns the XY Coordinates, given a node name
    def xyCoordinates(nodeName):
        if nodeName in T:
            return T[nodeName][0], T[nodeName][1]
        elif nodeName in D:
            return D[nodeName][0], D[nodeName][1]
        else:
            return "No node with this name found!"

    # In[51]:

    # Lets define our own tour cost function
    def tourCost(tour):
        costOfAllTours = 0
        # costOfEachTour = {k:pathLength(tour[k]) for k in K}
        for k in K:
            costOfAllTours += pathLength(tour[k])
        return costOfAllTours

    # In[52]:

    # Stochastic 2-opt
    def stochasticTwoOpt(perm, maxIter):
        _, fuelConsumedBefore2Opt = fuelCheck(perm, L)
        _, timeSpentBefore2Opt = timeCheck(perm, T_max)
        if len(perm) <= 3:
            print('No 2opt move possible with only %d nodes' % len(perm))
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
            print("Iteration: " + str(maxIter))
            print("2-Opt move attempted with edge (%s, %s) and (%s, %s)"
                  % (perm[p1], perm[p1 + 1], perm[p2 - 1], perm[p2]))
            print("New attempted path is :" + str(result))

            # Check if the path is fuel and time feasible
            fuelCheckPassed, fuelConsumedAfter2Opt = fuelCheck(result, L)
            timeCheckPassed, timeSpentAfter2Opt = timeCheck(result, T_max)

            if timeCheckPassed:
                # Check if the current time and fuel is less than previous ones
                if timeSpentAfter2Opt < timeSpentBefore2Opt:
                    # This is a valid 2opt
                    print("2-Opt move successful with edge (%s, %s) and (%s, %s)"
                          % (perm[p1], perm[p1 + 1], perm[p2 - 1], perm[p2]))
                    print("Previous Path: (Time Cost: %.2f)" %
                          timeSpentBefore2Opt)
                    pprint.pprint(perm)
                    print("New Path: (Time Cost: %.2f)" %
                          timeSpentAfter2Opt)
                    pprint.pprint(result)
                    return result

            maxIter -= 1
        print("No 2-Opt move feasible")
        pprint.pprint(perm)
        return perm

    # In[53]:

    # Local Seach Step
    def localSearch(best, maxIter):
        while maxIter > 0:
            candidate = {}
            candidate["permutation"] = stochasticTwoOpt(best["permutation"])
            candidate["cost"] = tourCost(candidate["permutation"])
            if candidate["cost"] < best["cost"]:
                best = candidate

            maxIter -= 1

        return best

    # In[54]:

    def insert(path, nonVisitedTasks):
        newPath = path[:]
        # Try to insert all the depots in the path that are not there
        for d in D:
            if d not in newPath:
                newPath, _ = checkadditionsInPath(newPath, d)
        # Try to insert all the nonVisitedTasks
        for t in nonVisitedTasks:
            newPath, _ = checkadditionsInPath(newPath, t)
        return newPath

    # In[55]:

    def nodesNotInTour(tour):
        nodesInTour = []
        for k in K:
            for n in N:
                if n in tour[k]:
                    nodesInTour.append(n)
        nodesNotInTour = [i for i in N if i not in nodesInTour]
        return nodesNotInTour

    def tasksNotInTour(tour):
        tasksInTour = []
        for k in K:
            for t in T:
                if t in tour[k]:
                    tasksInTour.append(t)
        tasksNotInTour = [i for i in T if i not in tasksInTour]
        return tasksNotInTour

    # In[56]:

    def pickTwoRandomPaths(tour):
        keys = rnd.sample(list(tour), 2)
        return keys[0], keys[1]

    # In[57]:

    def pickRandomNode(path):
        path = path[1:-1]
        index = rnd.randint(0, len(path) - 1)
        rndNode = path[index]
        index = index + 1  # Because we removed the first element from original path
        return rndNode, index

    # In[58]:

    def swap(tour, maxIter):
        while maxIter > 0:
            prevTour = copy.deepcopy(tour)
            # Swap tours from one path to the other path
            aK, bK = pickTwoRandomPaths(tour)
            _, fuelConsumedBeforeSwap1 = fuelCheck(tour[aK], L)
            _, fuelConsumedBeforeSwap2 = fuelCheck(tour[bK], L)
            _, timeSpentBeforeSwap1 = timeCheck(tour[aK], T_max)
            _, timeSpentBeforeSwap2 = timeCheck(tour[bK], T_max)
            fuelConsumedBeforeSwap = fuelConsumedBeforeSwap1 + fuelConsumedBeforeSwap2
            timeSpentBeforeSwap = timeSpentBeforeSwap1 + timeSpentBeforeSwap2
            print("Iteration: " + str(maxIter))
            # Pick two nodes randomly from each path, other than start nodes
            if timeSpentBeforeSwap1 > 0:
                node1, n1Idx = pickRandomNode(tour[aK])
            else:
                print("No Swap possible between paths of robots %s and %s."
                      % (aK, bK))
                continue
            if timeSpentBeforeSwap2 > 0:
                node2, n2Idx = pickRandomNode(tour[bK])
            else:
                print("No Swap possible between paths of robots %s and %s."
                      % (aK, bK))
                continue

            # Try swapping them in their paths
            newaK = copy.deepcopy(tour[aK])
            newbK = copy.deepcopy(tour[bK])
            newaK[n1Idx], newbK[n2Idx] = newbK[n2Idx], newaK[n1Idx]
            # print("Swap Attempted Between %s of path for robot %s and %s of path for robot %s "
            #                            % (node1, aK, node2, bK))

            # Check if the paths are fuel and time feasible
            fuelCheckPassed1, fuelConsumed1 = fuelCheck(newaK, L)
            fuelCheckPassed2, fuelConsumed2 = fuelCheck(newbK, L)
            fuelConsumedAfterSwap = fuelConsumed1 + fuelConsumed2

            timeCheckPassed1, timeSpent1 = timeCheck(newaK, T_max)
            timeCheckPassed2, timeSpent2 = timeCheck(newbK, T_max)
            timeSpentAfterSwap = timeSpent1 + timeSpent2

            if fuelCheckPassed1 and fuelCheckPassed2:
                if timeCheckPassed1 and timeCheckPassed2:
                    # Check if the current time and fuel is less than previous ones
                    if timeSpentAfterSwap < timeSpentBeforeSwap:
                        # This is a valid swap
                        # print("Swap Done Between %s of path for robot %s and %s of path for robot %s "
                        #                  % (node1, aK, node2, bK))
                        print("Previous Tour: (Time Cost: %.2f)" %
                              timeSpentBeforeSwap)
                        # pprint.pprint(prevTour)
                        tour[aK] = newaK[:]
                        tour[bK] = newbK[:]
                        print("New Tour: (Time Cost: %.2f)" %
                              timeSpentAfterSwap)
                        # pprint.pprint(tour)
                        return tour
            maxIter -= 1
        print("No swap executed:")
        print("Previous Tour:")
        pprint.pprint(prevTour)
        print("NewTour:")
        pprint.pprint(tour)
        return tour

    # In[59]:

    def move(tour):
        modifiedTour = copy.deepcopy(tour)
        # Arrange the tour paths in increasing length of paths
        sortedPathsByLength = sorted(tour.items(), key=lambda kv: len(kv[1]))
        sortedPathsByLength = [i for i, j in sortedPathsByLength]
        unexplored_paths = sortedPathsByLength[:]
        totalTimeBeforeMove = 0
        totalFuelConsumedBeforeMove = 0
        for k in K:
            _, temp = fuelCheck(tour[k], L)
            totalFuelConsumedBeforeMove += temp
            _, temp = timeCheck(tour[k], T_max)
            totalTimeBeforeMove += temp
        for k in sortedPathsByLength:
            unexplored_paths.remove(k)
            for node in tour[k][1:-1]:  # Skip the start and end nodes
                if node in N:
                    # Check if this node can be added to some other path by
                    # checking fuel and time checks
                    for k2 in unexplored_paths:
                        tempTour = copy.deepcopy(tour)
                        tempTour[k2], bool_c = checkadditionsInPath(
                            tour[k2], node)
                        if bool_c:
                            tempTour[k].remove(node)
                            totalFuelConsumedAfterMove = 0
                            totalTimeAfterMove = 0
                            for k3 in K:
                                _, temp = fuelCheck(tempTour[k3], L)
                                totalFuelConsumedAfterMove += temp
                                _, temp = timeCheck(tempTour[k3], T_max)
                                totalTimeAfterMove += temp
                            if totalTimeAfterMove < totalTimeBeforeMove:
                                modifiedTour = copy.deepcopy(tempTour)
                                # print("Node %s of path for robot %s moved to path of robot %s "
                                #        get_ipython().run_line_magic('(node,', 'k, k2))')
                                print("Previous Tour: (Time Cost: %.2f)" %
                                      totalTimeBeforeMove)
                                pprint.pprint(tour)
                                print("New Tour: (Time Cost: %.2f)" %
                                      totalTimeAfterMove)
                                pprint.pprint(modifiedTour)
                                break
        return modifiedTour
        # print ("No move operation beneficial.")
        # return tour

    # In[60]:

    # Attempt to remove extra depots
    def removeExtraDepots(tour):
        modfiedTour = copy.deepcopy(tour)
        for k in K:
            for node in tour[k]:
                tempPath = modfiedTour[k][:]
                if node in D and node != S[0]:
                    tempPath.remove(node)
                    fuelCheckPassed, _ = fuelCheck(tempPath, L)
                    timeCheckPassed, _ = timeCheck(tempPath, T_max)
                    if fuelCheckPassed and timeCheckPassed:
                        modfiedTour[k].remove(node)
                        print('Depot %s removed from path of robot %s' % (node, k))
        return modfiedTour

        # In[61]:

    # Perturbation
    def perturb(tour, A):
        modifiedTour = copy.deepcopy(tour)
        # compute no of depots and no of tasks in tour
        noOfDepotsInCurrSol = 0
        noOfTasksInCurrSol = 0
        for k in K:
            for node in tour[k]:
                if node in D and node != S[0]:
                    noOfDepotsInCurrSol += 1
                if node in T:
                    noOfTasksInCurrSol += 1

        rho = noOfDepotsInCurrSol / (noOfTasksInCurrSol + noOfDepotsInCurrSol)
        # Remove rho*A percent depots randomly from the current path
        # Remove (1-rho)*A percent tasks randomly from the current path
        for k in K:
            # Calculate indicies of depots and tasks in curr path
            depotIndiciesInCurrPath = []
            taskIndiciesInCurrPath = []
            for idx, node in enumerate(tour[k]):
                if node in D and node != S[0]:
                    depotIndiciesInCurrPath.append(idx)
                if node in T:
                    taskIndiciesInCurrPath.append(idx)
            # Now, remove rho*A percent depots from current path
            noOfDepotsToBeRemoved = math.ceil(0.3 * rho * A * len(depotIndiciesInCurrPath))
            print("%d depot(s) to be removed from path %s" % (noOfDepotsToBeRemoved, k))
            depotsInPath = [
                n for n in modifiedTour[k] if n in D and n != S[0]]
            if depotsInPath:
                for i in range(noOfDepotsToBeRemoved):
                    randomDepotIdx = rnd.choice(depotIndiciesInCurrPath)
                    randomDepot = tour[k][randomDepotIdx]
                    modifiedTour[k].remove(randomDepot)
                    depotIndiciesInCurrPath.remove(randomDepotIdx)
                    print("Depot %s removed from path of %s"
                          % (randomDepot, k))
            else:
                print("No Depot in path %s to remove." % k)
                # continue
            # Remove (1-rho)*A percent tasks from the current path
            noOfTasksToBeRemoved = math.ceil(0.3 * (1 - rho) * A * len(taskIndiciesInCurrPath))
            print("%d tasks to be removed from path %s" % (noOfTasksToBeRemoved, k))
            tasksInPath = [
                n for n in modifiedTour[k] if n in T and n != S[0]]
            if tasksInPath:
                for i in range(noOfTasksToBeRemoved):
                    randomTaskIdx = rnd.choice(taskIndiciesInCurrPath)
                    randomTask = tour[k][randomTaskIdx]
                    modifiedTour[k].remove(randomTask)
                    taskIndiciesInCurrPath.remove(randomTaskIdx)
                    print("Task %s removed from path of %s"
                          % (randomTask, k))
            else:
                print("No Task in path %s to remove." % k)
                # continue

        return modifiedTour

    # In[62]:

    def taskCount(tour):
        taskCountInTour = 0
        for k in K:
            for node in tour[k]:
                if node in T:
                    taskCountInTour += 1
        return taskCountInTour

    # In[63]:

    # Lets make a function that compares current solutoin with best solution
    def compareSolWithBest(currSol, bestSol):
        currSolTourCost = tourCost(currSol)
        bestSolTourCost = tourCost(bestSol)
        currSolTaskCount = taskCount(currSol)
        bestSolTaskCount = taskCount(bestSol)
        gamma = 0.0001
        currSolQuality = currSolTaskCount - gamma * currSolTourCost
        bestSolQuality = bestSolTaskCount - gamma * bestSolTourCost
        if currSolQuality > bestSolQuality:
            return currSol, True
        return bestSol, False

    # In[64]:

    def testFunc(sol):
        for k in K:
            arcsInOrderHeuristic[k] = pathArcRepresentation(sol[k])
        pprint.pprint(arcsInOrderHeuristic)
        fig = drawArena(T_loc, D_loc, arcsInOrderHeuristic, remainingFuel, 1)
        py.plot(fig, filename='temp3.html')

    # In[65]:

    # Main loop for Iterated Local Search

    def ILS():
        print("Random Seed:", thisSeed)
        diversificationParam = 1
        noImprovementIterations = 0
        maxIterations = 25
        # First convert the arc based path to nodeBased path
        candidateSolution = {}
        for k in K:
            candidateSolution[k] = pathNodeRepresentation(arcsInOrderHeuristic[k])
            # Lets not initialize it at all
            # candidateSolution[k] = [S[0], S[0]]
        # Initial Solution -------------------------------------------------------
        bestSolution = copy.deepcopy(candidateSolution)

        # Local Search  ----------------------------------------------------------
        # Insert
        nonVisitedTasks = tasksNotInTour(candidateSolution)
        for k in K:
            candidateSolution[k] = insert(candidateSolution[k], nonVisitedTasks)
            nonVisitedTasks = tasksNotInTour(candidateSolution)
        bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)
        # Swap
        candidateSolution = swap(candidateSolution, maxIterations)
        bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)
        # Move
        candidateSolution = move(candidateSolution)
        bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)
        # 2-opt
        for k in K:
            candidateSolution[k] = stochasticTwoOpt(candidateSolution[k], maxIterations)
        bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)
        # Remove Depots
        candidateSolution = removeExtraDepots(candidateSolution)
        bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)
        # 2-opt
        #    for k in K:
        #        candidateSolution[k] = stochasticTwoOpt(
        #            candidateSolution[k], maxIterations)
        #    bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)
        # Move
        candidateSolution = move(candidateSolution)
        bestSolution, _ = compareSolWithBest(candidateSolution, bestSolution)

        # Local Search Ended -----------------------------------------------------

        while noImprovementIterations < 10:
            print("No Improvement Itertion %d." % noImprovementIterations)
            print("Random Seed:", thisSeed)

            # Perturb
            candidateSolution = perturb(candidateSolution, diversificationParam)
            # Local Search  ----------------------------------------------------------
            # Insert
            nonVisitedTasks = tasksNotInTour(candidateSolution)
            for k in K:
                candidateSolution[k] = insert(
                    candidateSolution[k], nonVisitedTasks)
                nonVisitedTasks = tasksNotInTour(candidateSolution)
            bestSolution, boolImproved = compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0
            # Swap
            candidateSolution = swap(candidateSolution, maxIterations)
            bestSolution, boolImproved = compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0
            # Move
            candidateSolution = move(candidateSolution)
            bestSolution, boolImproved = compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0
            # 2-opt
            for k in K:
                candidateSolution[k] = stochasticTwoOpt(candidateSolution[k], maxIterations)
            bestSolution, boolImproved = compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0

            # Remove Depots
            candidateSolution = removeExtraDepots(candidateSolution)
            bestSolution, boolImproved = compareSolWithBest(
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
            candidateSolution = move(candidateSolution)
            bestSolution, boolImproved = compareSolWithBest(
                candidateSolution, bestSolution)
            if boolImproved:
                noImprovementIterations = 0

            noImprovementIterations += 1

            # Local Search Ended -----------------------------------------------------
            diversificationParam += 1
            # Check noOfnodes in the smallest path. We cannot remove more than those nodes
            smallestNoOfNodes = 1e6
            for k in K:
                noOfNodesInThisPath = len(candidateSolution[k])
                if noOfNodesInThisPath < smallestNoOfNodes:
                    smallestNoOfNodes = noOfNodesInThisPath
            if diversificationParam > smallestNoOfNodes:
                diversificationParam = diversificationParam - smallestNoOfNodes

        print('Best Solution:')
        pprint.pprint(bestSolution)
        return bestSolution

    # In[66]:

    # B = {'K0': ['D0', 'D1', 'D0'],
    #     'K1': ['D0', 'D2', 'T5', 'D0'],
    #     'K2': ['D0', 'D2', 'D2', 'D1', 'D0']}
    # B = removeExtraDepots(B)
    finalHeuristicSolution = ILS()

    # In[67]:

    for k in K:
        arcsInOrderHeuristic[k] = pathArcRepresentation(finalHeuristicSolution[k])
    pprint.pprint(arcsInOrderHeuristic)
    fig = drawArena(T_loc, D_loc, arcsInOrderHeuristic, remainingFuel, 1)
    py.iplot(fig)
    # py.plot(fig, filename='temp3.html')


def main():

    '''Run Heuristics with randomly generated input'''
    # Provide basic input
    noOfRobots = 3
    noOfTasks = 8
    noOfDepots = 2
    L = 200
    T_max = 500
    velocity = 1

    # randomly generated locations of tasks and robots
    K, T, D, S, T_loc, D_loc, N_loc = env.generate_test_instance_topf(noOfRobots, noOfTasks, noOfDepots)

    # Object of the planner
    milp = Greedy()


    # Plot the routes using plotly interactive GUI
    draw = Visualization_TOPF(plan, K, T, D, S, T_loc, D_loc, c)
    # filename if the plot to be saved
    name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots) + '.html'
    # plot and save
    auto_open_flag = 1
    draw.save_plot_topf(name, auto_open_flag)



if __name__ == "__main__":
    main()
