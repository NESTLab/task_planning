'''Moved as separated modules for each planner formulation'''


from gurobipy import *
import numpy as np


class TOPTW:
    def __init__(self):
        self.data = []

    '''Team orienteering problem with time windows'''
    def optimize(self, W, S, T, E, N_loc, noOfWorkerRobots, Q, O, C, D, T_max):
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




class TOPF:
    def __init__(self):
        self.data = []

    def optimize(self, K, S, T, D, N_loc, noOfRobots, noOfTasks, L, vel, T_max):
        N = T + D
        edges = [(i, j) for i in N for j in N if i != j]

        c = {t: np.linalg.norm(np.array(N_loc.get(t[0])) - np.array(N_loc.get(t[1]))) for t in iter(edges)}
        f = c

        arcs = [(i, j, k) for i in N for j in N for k in K if i != j]

        arc_ub = {(i, j, k): 1 for i in N for j in N for k in K if i != j}
        for arc in arc_ub:
            if arc[0] in D and arc[1] in D:
                arc_ub[arc] = noOfTasks

        k_y = [(i, k) for i in T for k in K]

        model = Model('OrienteeringFuelConstrainedRobots')
        x = model.addVars(arcs, lb=0, ub=arc_ub, name="x", vtype=GRB.INTEGER)
        y = model.addVars(k_y, name="y", vtype=GRB.BINARY)
        r = model.addVars(T, lb=0, ub=L, vtype=GRB.CONTINUOUS, name="r")
        p = model.addVars(arcs, name="p", vtype=GRB.INTEGER)

        gamma = 0.00001
        objExpr1 = quicksum(y[i, k] for i in T for k in K)
        objExpr2 = quicksum(gamma * c[i, j] * x[i, j, k] for k in K for i in N for j in N if i != j)

        objFun = objExpr1 - objExpr2
        model.setObjective(objFun, GRB.MAXIMIZE)

        c2 = model.addConstr((quicksum(x[s, j, k] for j in N for k in K for s in S if j not in S) == noOfRobots),
                             name="c2")

        c31 = model.addConstrs((quicksum(x[i, e, k] for i in N for k in K if i not in S) == noOfRobots for e in S),
                               name="c3")

        c8 = model.addConstrs(((quicksum(x[s, j, k] for s in S for j in N if s != j)) <= 1 for k in K), name="c8")
        c9 = model.addConstrs(((quicksum(x[i, s, k] for s in S for i in N if i != s)) <= 1 for k in K), name="c9")

        c4 = model.addConstrs((quicksum(y[i, k] for k in K) <= 1 for i in T), name="c4")

        c51 = model.addConstrs(((quicksum(x[i, h, k] for i in N if i != h)) ==
                                (quicksum(x[h, j, k] for j in N if j != h)) for h in N for k in K), name="c5_1")

        c52 = model.addConstrs(((quicksum(x[i, h, k] for i in N if i != h)) ==
                                y[h, k] for h in T for k in K), name="c5_2")

        c53 = model.addConstrs(((quicksum(x[h, j, k] for j in N if j != h)) ==
                                y[h, k] for h in T for k in K), name="c5_3")

        M = 1e6
        c15 = model.addConstrs(
            (r[j] - r[i] + f[i, j] <= M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c15")
        c16 = model.addConstrs(
            (r[j] - r[i] + f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c16")

        c18 = model.addConstrs((r[j] - L + f[i, j] <= M * (1 - x[i, j, k]) for i in D for j in T for k in K),
                               name="c18")
        c17 = model.addConstrs((r[j] - L + f[i, j] >= -M * (1 - x[i, j, k]) for i in D for j in T for k in K),
                               name="c17")

        c19 = model.addConstrs((r[i] - f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in D for k in K), name="c19")

        c22 = model.addConstrs((f[i, j] * x[i, j, k] <= L for i in D for j in D for k in K if i != j), name="c22")

        c20 = model.addConstrs((0 <= r[i] <= L for i in T), name="c20")

        c21 = model.addConstrs(((quicksum(f[i, j] * x[i, j, k] for i in N for j in N if i != j)) <=
                                L * (quicksum(x[d, i, k] for i in N for d in D if d != i)) for k in K), name="c21")

        c23 = model.addConstrs(
            (quicksum(c[i, j] * x[i, j, k] / vel for i in N for j in N if i != j and i not in S and j not in S) <= T_max
             for k in K), name="c23")

        model.optimize()

        return model







