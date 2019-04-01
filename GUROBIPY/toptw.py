'''
MILP Formulation for Team Orienteering Problem with Time Windows

This script requires that
    *'gurobipy'
    *'numpy'
be installed within the Python environment you are running this script in.

This file can also be imported as a module and contains the following functions:
    * planner - finds an optimized path given certain constraints
'''


# Importing existing python modules
from gurobipy import *
import numpy as np

# Importing required project modules
import environment as env


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





def main():
    '''Run TOPF with randomly generated input'''
    # Provide basic input
    noOfTasks = 8
    noOfWorkerRobots = 4
    noOfStartNodes = 1
    maxTimeInterval = 50
    maxStartTime = 400 - maxTimeInterval
    maxTaskDuration = 10
    velocity = 1; T_max = 1000

    # Read input -> Plan -> Save computational data (.csv)
    # randomly generated locations of tasks and robots
    W, S, T, E, N_loc, Q, O, C, D = env.generate_test_instance_toptw(noOfWorkerRobots, noOfTasks, noOfStartNodes,
                                                               maxTaskDuration, T_max, maxTimeInterval)
    # Object of the planner
    milp = TOPTW(velocity)
    # Optimize!!!
    milp.planner(W, S, T, E, N_loc, noOfWorkerRobots, Q, O, C, D, T_max)


if __name__ == "__main__":
    main()