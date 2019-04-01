'''
MILP Formulation for Team Orienteering Problem with Fuel Constrained Robots

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
from visualization import Visualization_TOPF


class TOPF:
    def __init__(self, velocity):
        self.vel = velocity


    def planner(self, K, T, D, S, N_loc, noOfRobots, noOfTasks, L, T_max):
        '''
        Formulation without flow and capacity constraints
        :param K:
        :param T:
        :param D:
        :param S:
        :param N_loc:
        :param noOfRobots:
        :param noOfTasks:
        :param L:
        :param T_max:
        :return: Optimized model
        '''

        '''Pre-processing'''
        # Total nodes
        N = T + D
        # Defining edges of the graph
        edges = [(i, j) for i in N for j in N if i != j]

        # Distance between nodes = edge weight
        c = {t: np.linalg.norm(np.array(N_loc.get(t[0])) - np.array(N_loc.get(t[1]))) for t in iter(edges)}
        f = c


        # Defining arcs for decision variables
        arcs = [(i, j, k) for i in N for j in N for k in K if i != j]

        arc_ub = {(i, j, k): 1 for i in N for j in N for k in K if i != j}
        for arc in arc_ub:
            if arc[0] in D and arc[1] in D:
                arc_ub[arc] = noOfTasks

        k_y = [(i, k) for i in T for k in K]


        '''Define the model and decision variables'''
        # Initialize the model
        model = Model('OrienteeringFuelConstrainedRobots')
        # Decision variables and their bounds
        x = model.addVars(arcs, lb=0, ub=arc_ub, name="x", vtype=GRB.INTEGER)
        y = model.addVars(k_y, name="y", vtype=GRB.BINARY)
        r = model.addVars(T, lb=0, ub=L, vtype=GRB.CONTINUOUS, name="r")

        # Objective function
        gamma = 0.00001
        objExpr1 = quicksum(y[i, k] for i in T for k in K)
        objExpr2 = quicksum(gamma * c[i, j] * x[i, j, k] for k in K for i in N for j in N if i != j)

        objFun = objExpr1 - objExpr2
        model.setObjective(objFun, GRB.MAXIMIZE)


        '''Constraints'''

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
            (quicksum(c[i, j] * x[i, j, k] / self.vel for i in N for j in N if i != j and i not in S and j not in S) <= T_max
             for k in K), name="c23")


        # Optimize
        model.optimize()

        return c, model




def main():

    '''Run TOPF with randomly generated input'''
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
    milp = TOPF(velocity)
    # Optimize model
    c, plan = milp.planner(K, T, D, S, N_loc, noOfRobots, noOfTasks, L, T_max)

    # Plot the routes using plotly interactive GUI
    draw = Visualization_TOPF(plan, K, T, D, S, T_loc, D_loc, c)
    # filename if the plot to be saved
    name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots) + '.html'
    # plot and save
    auto_open_flag = 1
    draw.save_plot_topf(name, auto_open_flag)



if __name__ == "__main__":
    main()