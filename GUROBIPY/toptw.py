#######################################################################################################################
                                         # TOP with time windows #
#######################################################################################################################


from gurobipy import *
import numpy as np
import random as rnd
import itertools
import csv


class TOPTW:
    def __init__(self, velocity):
        self.vel = velocity


    def planner(self, W, S, T, E, N_loc, noOfWorkerRobots, Q, O, C, D, T_max):
        '''Pre-processing'''
        # Nodes and arcs
        N = S + T + E
        arcs = [(i, j) for i in N for j in N if i != j]

        # Distance between each node aka weights of the arcs
        iter_arcs = iter(arcs)
        t_arcs = {t: np.linalg.norm(np.array(N_loc.get(t[0])) - np.array(N_loc.get(t[1]))) for t in iter_arcs}

        w_arcs = [(w, arc) for w in W for arc in arcs]

        '''Decision variables'''
        w_y = T
        s_ik = [(w, t) for w in W for t in T]

        model = Model('FuelConstrainedRobots')
        x = model.addVars(w_arcs, name="x", vtype=GRB.BINARY)
        s = model.addVars(s_ik, name="s")
        y = model.addVars(w_y, name="y", vtype=GRB.BINARY)

        '''Objective function'''
        objFun = quicksum(y[i] for i in T)
        model.setObjective(objFun, GRB.MAXIMIZE)


        '''Constraints'''

        c2 = model.addConstrs((quicksum(x[k, (s, j)] for j in T + E for k in W) == noOfWorkerRobots for s in S),
                              name="c2_1")
        c22 = model.addConstrs((quicksum(x[k, (s, j)] for j in T + E) == 1 for s in S for k in W), name="c2_2")

        c31 = model.addConstrs((quicksum(x[k, (i, e)] for i in S + T for k in W) == noOfWorkerRobots for e in E), name="c3")

        c41 = model.addConstrs((y[i] <= 1 for i in T), name="c4_1")
        c42 = model.addConstrs((quicksum(x[k, (i, j)] for k in W for i in S + T if i != j) == Q[j] * y[j] for j in T),
                               name="c4_2")
        c51 = model.addConstrs(((quicksum(x[k, (i, h)] for i in S + T if i != h)) ==
                                (quicksum(x[k, (h, j)] for j in T + E if j != h)) for h in T for k in W), name="c5_1")

        c52 = model.addConstrs(((quicksum(x[k, (i, h)] for i in S + T if i != h)) <= y[h] for k in W for h in T),
                               name="c5_2")

        c53 = model.addConstrs(((quicksum(x[k, (h, j)] for j in T + E if j != h)) <= y[h] for k in W for h in T),
                               name="c5_3")

        c6 = model.addConstrs(
            (quicksum((t_arcs.get((i, j)) + D[i]) * x[k, (i, j)] for i in S + T for j in T + E if i != j) <= T_max
             for k in W), name="c6")

        M = 1e6
        c8 = model.addConstrs(
            (s[k, i] + t_arcs.get((i, j)) + D[i] - s[k, j] <= M * (1 - x[k, (i, j)]) for i in T for j in T for k in W if
             i != j), name="c8")

        c91 = model.addConstrs(((O[i] <= s[k, i]) for i in T for k in W), name="c9_1")
        c92 = model.addConstrs(((s[k, i] <= C[i]) for i in T for k in W), name="c9_2")





        '''Optimize'''
        model.optimize()

        return model


    def generate_test_instance(self, noOfWorkerRobots, noOfTasks, noOfStartNodes, maxTaskDuration, maxStartTime, maxTimeInterval):
        # Set of robots
        W = ["W" + str(i) for i in range(noOfWorkerRobots)]
        # Set of task nodes
        T = ["T" + str(i) for i in range(noOfTasks)]
        # Set of start nodes
        S = ["S" + str(i) for i in range(noOfStartNodes)]
        # Set of end nodes
        E = ["E"]

        U = [i for i in T + E]

        # Set of task duration
        D = {t: rnd.randint(1, maxTaskDuration) for t in T}
        D["S0"] = 0
        # Set of quota requirements
        Q = {t: rnd.randint(1, noOfWorkerRobots - 1) for t in T}

        # Set of maximum start times
        O = {t: rnd.randint(1, maxStartTime) for t in T}
        # Set of task end times
        EndTime = {t: rnd.randint(1, maxTimeInterval) + O[t] for t in T}
        C = {t: (EndTime[t] - D[t]) for t in T}
        # Activation Window
        A = {t: (O[t], EndTime[t]) for t in T}

        # Task Locations
        T_loc = {task: (100 * rnd.random(), 100 * rnd.random()) for task in T}

        # start and end locations
        S_loc = {loc: (100 * rnd.random(), 100) for loc in S}
        E_loc = {loc: (100 * rnd.random(), 100) for loc in E}

        N_loc = {**S_loc, **T_loc, **E_loc}

        return W, S, T, E, N_loc, Q, O, C, D

    def collect_data(self, robots_range, task_range, Tmax_range, noOfStartNodes, maxTaskDuration, T_max, maxTimeInterval):
        for T_max, noOfWorkerRobots, noOfTasks in itertools.product(Tmax_range, robots_range, task_range):

            print("********************************************************")
            print(noOfWorkerRobots, noOfTasks, T_max)
            runtime = 0
            while(runtime < 0.01):
                # Generate feasible environments
                W, S, T, E, N_loc, Q, O, C, D = self.generate_test_instance(noOfWorkerRobots, noOfTasks, noOfStartNodes, maxTaskDuration, T_max, maxTimeInterval)
                # Run planner
                plan = self.planner(W, S, T, E, N_loc, noOfWorkerRobots, Q, O, C, D, T_max)

                runtime = plan.Runtime
                # Save data
                row = [noOfWorkerRobots, noOfTasks, T_max, runtime]

                with open('experiment1.csv', 'a') as csvFile:
                    writer = csv.writer(csvFile)
                    writer.writerow(row)

                csvFile.close()


def main():
    ## single case
    noOfTasks = 8
    noOfWorkerRobots = 4
    noOfStartNodes = 1
    maxTimeInterval = 50
    maxStartTime = 400 - maxTimeInterval
    maxTaskDuration = 10
    velocity = 1
    T_max = 1000
    
    trial = TOPTW(velocity)
    W, S, T, E, N_loc, Q, O, C, D = trial.generate_test_instance(noOfWorkerRobots, noOfTasks, noOfStartNodes, maxTaskDuration, T_max, maxTimeInterval)
    trial.planner(W, S, T, E, N_loc, noOfWorkerRobots, Q, O, C, D, T_max);

    ## experiments
    # robots_range = [3, 4]
    # task_range = [7]
    # Tmax_range = [300]
    # velocity = 1
    #
    # trial = TOPTW(velocity)
    # trial.collect_data(robots_range, task_range, Tmax_range, noOfStartNodes, maxTaskDuration, maxStartTime, maxTimeInterval)


if __name__ == "__main__":
    main()