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


    def planner(self, K, T, D, S, E, N_loc, noOfRobots, noOfTasks, L, T_max, R):
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

        S = ['D0']
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
        gamma = 0.0001
        objExpr1 = quicksum(y[i, k] for i in T for k in K)
        objExpr2 = quicksum(gamma * c[i, j] * x[i, j, k] for k in K for i in N for j in N if i != j)

        objFun = objExpr1 - objExpr2
        model.setObjective(objFun, GRB.MAXIMIZE)


        '''Constraints'''

        # 1. each robot starts at start node
        c1 = model.addConstr((quicksum(x[s, j, k] for j in N for k in K for s in S if j not in S) == noOfRobots),
                             name="c1")

        # 2. each robot ends at end node
        c2 = model.addConstrs((quicksum(x[i, e, k] for i in N for k in K if i not in S) == noOfRobots for e in S),
                               name="c2")

        # 3. each robot begins and ends at start location
        c31 = model.addConstrs(((quicksum(x[s, j, k] for s in S for j in N if s != j)) <= 1 for k in K), name="c31")
        c32 = model.addConstrs(((quicksum(x[i, s, k] for s in S for i in N if i != s)) <= 1 for k in K), name="c32")

        # 4. each task visited once or never
        c4 = model.addConstrs((quicksum(y[i, k] for k in K) <= 1 for i in T), name="c4")


        # 5. robot enters a node, leaves a node
        c51 = model.addConstrs(((quicksum(x[i, h, k] for i in N if i != h)) ==
                                (quicksum(x[h, j, k] for j in N if j != h)) for h in N for k in K), name="c5_1")

        c52 = model.addConstrs(((quicksum(x[i, h, k] for i in N if i != h)) ==
                                y[h, k] for h in T for k in K), name="c5_2")

        c53 = model.addConstrs(((quicksum(x[h, j, k] for j in N if j != h)) ==
                                y[h, k] for h in T for k in K), name="c5_3")

        '''fuel constraints'''
        # 6. Ensure fuel conservation when the UAV travels between two targets
        M = 1e6
        c61 = model.addConstrs(
            (r[j] - r[i] + f[i, j] <= M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c61")
        c62 = model.addConstrs(
            (r[j] - r[i] + f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c62")

        # 7. Establish the condition that the fuel level at a target visited after leaving a depot
        # is equal to the fuel capacity minus the fuel cost of traversal
        c71 = model.addConstrs((r[j] - L + f[i, j] <= M * (1 - x[i, j, k]) for i in D for j in T for k in K),
                               name="c71")
        c72 = model.addConstrs((r[j] - L + f[i, j] >= -M * (1 - x[i, j, k]) for i in D for j in T for k in K),
                               name="c72")

        # 8. Restrict the fuel lost in approaching a depot to being at most equal to the
        # cost of travel from the preceding target
        c8 = model.addConstrs((r[i] - f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in D for k in K), name="c8")

        # 9. Restricts direct paths between refueling sites to exist only between sites at most 𝑈 distance away
        c9 = model.addConstrs((f[i, j] * x[i, j, k] <= L for i in D for j in D for k in K if i != j), name="c9")

        # 10. fuel level parameter to be bounded between 0 and L
        c10 = model.addConstrs((0 <= r[i] <= L for i in T), name="c10")

        # 11. Total fuel consumed by the UAV must be less than or equal to  𝐿  times the total number of refueling visits
        c11 = model.addConstrs(((quicksum(f[i, j] * x[i, j, k] for i in N for j in N if i != j)) <=
                                L * (quicksum(x[d, i, k] for i in N for d in D if d != i)) for k in K), name="c11")

        # 12. Ensure that each robot is back at the start location before 𝑇𝑚𝑎𝑥
        c12 = model.addConstrs(
            (quicksum(c[i, j] * x[i, j, k] / self.vel for i in N for j in N if i != j and i not in S and j not in S) <= T_max
             for k in K), name="c12")


        # Optimize
        model.params.Heuristics = 0 # Do not use a heuristic solution
        model.params.Cuts = 0 # Do not use cuts, except lazy constraints
        model.optimize()

        return c, model



    def planner_with_flow(self, K, T, D, S, E, N_loc, noOfRobots, noOfTasks, L, T_max, R):
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
        p = model.addVars(arcs, name="p", vtype=GRB.INTEGER)

        # Objective function
        gamma = 0.0001
        objExpr1 = quicksum(y[i, k] for i in T for k in K)
        objExpr2 = quicksum(gamma * c[i, j] * x[i, j, k] for k in K for i in N for j in N if i != j)

        objFun = objExpr1 - objExpr2
        model.setObjective(objFun, GRB.MAXIMIZE)


        '''Constraints'''

        # 1. each robot starts at start node
        c1 = model.addConstr((quicksum(x[s, j, k] for j in N for k in K for s in S if j not in S) == noOfRobots),
                             name="c1")

        # 2. each robot ends at end node
        c2 = model.addConstrs((quicksum(x[i, e, k] for i in N for k in K if i not in S) == noOfRobots for e in S),
                               name="c2")

        # 3. each robot begins and ends at start location
        c31 = model.addConstrs(((quicksum(x[s, j, k] for s in S for j in N if j not in S)) == 1 for k in K),
                                name="c31")
        c32 = model.addConstrs(((quicksum(x[i, e, k] for e in E for i in N if i not in E)) == 1 for k in K),
                                name="c32")


        # 4. each task visited once or never
        c4 = model.addConstrs((quicksum(y[i, k] for k in K) <= 1 for i in T), name="c4")


        # 5. robot enters a node, leaves a node
        c51 = model.addConstrs(((quicksum(x[i, h, k] for i in N if i != h)) ==
                                (quicksum(x[h, j, k] for j in N if j != h)) for h in N for k in K), name="c5_1")

        c52 = model.addConstrs(((quicksum(x[i, h, k] for i in N if i != h)) ==
                                y[h, k] for h in T for k in K), name="c5_2")

        c53 = model.addConstrs(((quicksum(x[h, j, k] for j in N if j != h)) ==
                                y[h, k] for h in T for k in K), name="c5_3")

        '''fuel constraints'''
        # 6. Ensure fuel conservation when the UAV travels between two targets
        M = 1e6
        c61 = model.addConstrs(
            (r[j] - r[i] + f[i, j] <= M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c61")
        c62 = model.addConstrs(
            (r[j] - r[i] + f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c62")

        # 7. Establish the condition that the fuel level at a target visited after leaving a depot
        # is equal to the fuel capacity minus the fuel cost of traversal
        c71 = model.addConstrs((r[j] - L + f[i, j] <= M * (1 - x[i, j, k]) for i in D for j in T for k in K),
                               name="c71")
        c72 = model.addConstrs((r[j] - L + f[i, j] >= -M * (1 - x[i, j, k]) for i in D for j in T for k in K),
                               name="c72")

        # 8. Restrict the fuel lost in approaching a depot to being at most equal to the
        # cost of travel from the preceding target
        c8 = model.addConstrs((r[i] - f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in D for k in K), name="c8")

        # 9. Restricts direct paths between refueling sites to exist only between sites at most 𝑈 distance away
        c9 = model.addConstrs((f[i, j] * x[i, j, k] <= L for i in D for j in D for k in K if i != j), name="c9")

        # 10. fuel level parameter to be bounded between 0 and L
        c10 = model.addConstrs((0 <= r[i] <= L for i in T), name="c10")

        # 11. Total fuel consumed by the UAV must be less than or equal to  𝐿  times the total number of refueling visits
        c11 = model.addConstrs(((quicksum(f[i, j] * x[i, j, k] for i in N for j in N if i != j)) <=
                                L * (quicksum(x[d, i, k] for i in N for d in D if d != i)) for k in K), name="c11")

        # 12. Ensure that each robot is back at the start location before 𝑇𝑚𝑎𝑥
        c12 = model.addConstrs(
            (quicksum(c[i, j] * x[i, j, k] / self.vel for i in N for j in N if i != j and i not in S and j not in S) <= T_max
             for k in K), name="c12")

        '''capacity & flow constraints'''
        # 13. total capacity at start
        c13 = model.addConstrs(((quicksum((p[s, i, k] - p[i, s, k]) for i in N for s in S if i not in S)) ==
                                                        (quicksum(x[i,j,k] for i in T for j in N if i!=j and i not in S)) for k in K), name="c13")

        # 14. capacity reduced by 1
        c14 = model.addConstrs(((quicksum((p[j,i,k] - p[i,j,k]) for j in N if i!=j)) ==
                                (quicksum(x[i,j,k] for j in N if i!=j )) for i in T for k in K if i not in S), name="c14")

        # 15. As the robot passes through refueling depots, though, this target capacity is prevented from changing
        c15 = model.addConstrs(((quicksum((p[j, i, k] - p[i, j, k]) for j in N if j != i)) == 0 for i in D for k in K if i not in S), name="c15")

        # 16. target capacity for each robot does not exceed |𝑇|
        c16 = model.addConstrs((0 <= p[i, j, k] <= noOfTasks * x[i, j, k] for i in N for j in N for k in K if i != j),
                               name="c16")

        # Optimize
        model.params.Heuristics = 0 # Do not use a heuristic solution
        model.params.Cuts = 0 # Do not use cuts, except lazy constraints
        model.optimize()

        return c, model



    def new_planner(self, K, T, D, S, E, N_loc, noOfRobots, noOfTasks, L, T_max, R):
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
        N = T + D + S + E
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
            if arc[0] in E and arc[1] in S:
                arc_ub[arc] = 0  # You cannot go from end to start

        #k_y = [(i, k) for i in T for k in K]


        '''Define the model and decision variables'''
        # Initialize the model
        model = Model('TO')
        # Decision variables and their bounds
        x = model.addVars(arcs, lb=0, ub=arc_ub, name="x", vtype=GRB.INTEGER)
        y = model.addVars(T, name="y", vtype=GRB.BINARY)
        r = model.addVars(T, lb=0, ub=L, vtype=GRB.CONTINUOUS, name="r")
        p = model.addVars(arcs, name="p", vtype=GRB.INTEGER)

        # Objective function
        gamma = 0.0001
        # rewards
        objExpr1 = quicksum(y[i] for i in T)
        objExpr2 = quicksum(gamma * c[i, j] * x[i, j, k] for k in K for i in N for j in N if i != j)

        objFun = objExpr1 - objExpr2
        model.setObjective(objFun, GRB.MAXIMIZE)


        '''Constraints'''

        # 1. each robot starts at start node
        c1 = model.addConstr((quicksum(x[s, j, k] for j in N for k in K for s in S if j not in S) == noOfRobots),name="c1")

        # 2. each robot ends at end node
        c2 = model.addConstr((quicksum(x[i,e,k] for i in N for k in K for e in E if i!=e) == noOfRobots), name="c2")

        # 3. each robot begins and ends at start location
        c31 = model.addConstrs(((quicksum(x[s, j, k] for s in S for j in N if j not in S)) == 1 for k in K), name="c31")
        c32 = model.addConstrs(((quicksum(x[j, s, k] for s in S for j in N if j not in S)) == 0 for k in K), name="c32")

        c33 = model.addConstrs(((quicksum(x[i, e, k] for e in E for i in N if i not in E)) == 1 for k in K), name="c33")
        c34 = model.addConstrs(((quicksum(x[e, i, k] for e in E for i in N if i not in E)) == 0 for k in K),name="c34")

        # model.write("x.lp")
        # 4. each task visited once or never
        c4 = model.addConstrs((y[i] <= 1 for i in T), name="c4")


        # 5. robot enters a node, leaves a node
        c51 = model.addConstrs(((quicksum(x[i, h, k] for i in N if i != h and i not in E)) ==
                                (quicksum(x[h, j, k] for j in N if j != h and j not in S))
                                for h in N for k in K if h not in S and h not in E), name="c5_1")

        c52 = model.addConstrs(((quicksum(x[i, h, k] for k in K for i in N if i!=h)) == 
                                    y[h] for h in T), name="c5_2")

        c53 = model.addConstrs(((quicksum(x[h, j, k] for k in K for j in N if j!=h)) == 
                                    y[h] for h in T), name="c5_3")

        '''fuel constraints'''
        # 6. Ensure fuel conservation when the UAV travels between two targets
        M = 1e6
        c61 = model.addConstrs(
            (r[j] - r[i] + f[i, j] <= M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c61")
        c62 = model.addConstrs(
            (r[j] - r[i] + f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c62")

        # 7. Establish the condition that the fuel level at a target visited after leaving a  (or a start node)
        # is equal to the fuel capacity minus the fuel cost of traversal
        c71 = model.addConstrs((r[j] - L + f[i, j] <= M * (1 - x[i, j, k]) for i in D+S for j in T for k in K), name="c71")
        c72 = model.addConstrs((r[j] - L + f[i, j] >= -M * (1 - x[i, j, k]) for i in D+S for j in T for k in K), name="c72")

        # 8. Restrict the fuel lost in approaching a depot (or an end node) to being at most equal to the
        # cost of travel from the preceding target
        c8 = model.addConstrs((r[i] - f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in D+E for k in K), name="c8")

        # 9. Restricts direct paths between refueling sites (or the start/end nodes) to exist only between sites at most 𝑈 distance away
        c9 = model.addConstrs((f[i, j] * x[i, j, k] <= L for i in D+S+E for j in D+S+E for k in K if i != j), name="c9")

        # 10. fuel level parameter to be bounded between 0 and L
        c10 = model.addConstrs((0 <= r[i] <= L for i in T), name="c10")

        # 11. Total fuel consumed by the UAV must be less than or equal to  𝐿  times the total number of refueling visits
        #c11 = model.addConstrs(((quicksum(f[i, j] * x[i, j, k] for i in N for j in N if i != j)) <=
        #                        L * (quicksum(x[d, i, k] for i in N for d in D if d != i)) for k in K), name="c11")

        # 12. Ensure that each robot is back at the start location before 𝑇𝑚𝑎𝑥
        c12 = model.addConstrs(
            (quicksum(c[i, j] * x[i, j, k] / self.vel for i in N for j in N if i != j) <= T_max
             for k in K), name="c12")

        '''capacity & flow constraints'''
        # 13. total capacity at start
        c13 = model.addConstrs(((quicksum((p[s, i, k] - p[i, s, k]) for i in N for s in S if i not in S)) ==
                                                        (quicksum(x[i,j,k] for i in T for j in N if i!=j and i not in S)) for k in K), name="c13")

        # 14. capacity reduced by 1
        c14 = model.addConstrs(((quicksum((p[j,i,k] - p[i,j,k]) for j in N if i!=j)) ==
                                (quicksum(x[i,j,k] for j in N if i!=j )) for i in T for k in K if i not in S), name="c14")

        # 15. As the robot passes through refueling depots, though, this target capacity is prevented from changing
        c15 = model.addConstrs(((quicksum((p[j, i, k] - p[i, j, k]) for j in N if j != i)) == 0 for i in D for k in K if i not in S), name="c15")

        # 16. target capacity for each robot does not exceed |𝑇|
        c16 = model.addConstrs((0 <= p[i, j, k] <= noOfTasks * x[i, j, k] for i in N for j in N for k in K if i != j),name="c16")

        # Optimize
        #model.params.Heuristics = 0 # Do not use a heuristic solution
        #model.params.Cuts = 0 # Do not use cuts, except lazy constraints
        model.params.TimeLimit = 30 # only run exploration for 30 seconds
        model.optimize()

        return c, model


    def new_planner_minmax(self, K, T, D, S, E, N_loc, noOfRobots, noOfTasks, L, T_max, R):
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
        N = T + D + S + E
        # Defining edges of the graph
        edges = [(i, j) for i in N for j in N if i != j]

        # Distance between nodes = edge weight
        c = {t: np.linalg.norm(
            np.array(N_loc.get(t[0])) - np.array(N_loc.get(t[1]))) for t in iter(edges)}
        f = c

        # Defining arcs for decision variables
        arcs = [(i, j, k) for i in N for j in N for k in K if i != j]

        arc_ub = {(i, j, k): 1 for i in N for j in N for k in K if i != j}
        for arc in arc_ub:
            if arc[0] in D and arc[1] in D:
                arc_ub[arc] = noOfTasks
            if arc[0] in E and arc[1] in S:
                arc_ub[arc] = 0  # You cannot go from end to start

        k_y = [(i, k) for i in T for k in K]

        '''Define the model and decision variables'''
        # Initialize the model
        model = Model('TOMinMax')
        # Decision variables and their bounds
        x = model.addVars(arcs, lb = 0, ub = arc_ub, name="x", vtype=GRB.INTEGER)
        y = model.addVars(k_y, name="y", vtype=GRB.BINARY)
        r = model.addVars(T, lb=0, ub=L, vtype=GRB.CONTINUOUS, name="r")
        p = model.addVars(arcs, name="p", vtype=GRB.INTEGER)
        z = model.addVar(name="z", vtype=GRB.CONTINUOUS)

        # Objective function
        gamma = 1e-4
        # rewards
        # objExpr1 = quicksum(y[i] for i in T)
        objExpr1 = z
        objExpr2 = quicksum(gamma * c[i, j] * x[i, j, k]
                            for k in K for i in N for j in N if i != j)

        objFun = objExpr1 - objExpr2
        model.setObjective(objFun, GRB.MAXIMIZE)

        '''Constraints'''

        # 0. Add the constraint that balances task assignment for each robot.
        c0 = model.addConstrs((quicksum(y[i,k] for i in T) >= z for k in K), name="c0")

        # 1. each robot starts at start node
        c1 = model.addConstr((quicksum(
            x[s, j, k] for j in N for k in K for s in S if j not in S) == noOfRobots), name="c1")

        # 2. each robot ends at end node
        c2 = model.addConstr((quicksum(
            x[i, e, k] for i in N for k in K for e in E if i != e) == noOfRobots), name="c2")

        # 3. each robot begins and ends at start location
        c31 = model.addConstrs(((quicksum(
            x[s, j, k] for s in S for j in N if j not in S)) == 1 for k in K), name="c31")
        c32 = model.addConstrs(((quicksum(
            x[j, s, k] for s in S for j in N if j not in S)) == 0 for k in K), name="c32")

        c33 = model.addConstrs(((quicksum(
            x[i, e, k] for e in E for i in N if i not in E)) == 1 for k in K), name="c33")
        c34 = model.addConstrs(((quicksum(
            x[e, i, k] for e in E for i in N if i not in E)) == 0 for k in K), name="c34")

        # model.write("x.lp")
        # 4. each task visited once or never
        c4 = model.addConstrs((quicksum(y[i,k] for k in K) <= 1 for i in T), name="c4")

        # 5. robot enters a node, leaves a node, and enter and leave only active ones.
        c51 = model.addConstrs(((quicksum(x[i,h,k] for i in N if i!=h and i not in E)) == 
                                (quicksum(x[h,j,k] for j in N if j!=h and j not in S)) 
                                                    for h in N for k in K if h not in S and h not in E), name="c5_1")

        c52 = model.addConstrs(((quicksum(x[i,h,k] for i in N if i!=h)) == 
                                            y[h,k] for h in T for k in K), name="c5_2")

        c53 = model.addConstrs(((quicksum(x[h,j,k] for j in N if j!=h)) == 
                                            y[h,k] for h in T for k in K), name="c5_3")

        '''fuel constraints'''
        # 6. Ensure fuel conservation when the UAV travels between two targets
        M = 1e6
        c61 = model.addConstrs(
            (r[j] - r[i] + f[i, j] <= M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c61")
        c62 = model.addConstrs(
            (r[j] - r[i] + f[i, j] >= -M * (1 - x[i, j, k]) for i in T for j in T for k in K if i != j), name="c62")

        # 7. Establish the condition that the fuel level at a target visited after leaving a  (or a start node)
        # is equal to the fuel capacity minus the fuel cost of traversal
        c71 = model.addConstrs((r[j] - L + f[i, j] <= M * (1 - x[i, j, k])
                                for i in D+S for j in T for k in K), name="c71")
        c72 = model.addConstrs((r[j] - L + f[i, j] >= -M * (1 - x[i, j, k])
                                for i in D+S for j in T for k in K), name="c72")

        # 8. Restrict the fuel lost in approaching a depot (or an end node) to being at most equal to the
        # cost of travel from the preceding target
        c8 = model.addConstrs((r[i] - f[i, j] >= -M * (1 - x[i, j, k])
                               for i in T for j in D+E for k in K), name="c8")

        # 9. Restricts direct paths between refueling sites (or the start/end nodes) to exist only between sites at most 𝑈 distance away
        c9 = model.addConstrs(
            (f[i, j] * x[i, j, k] <= L for i in D+S+E for j in D+S+E for k in K if i != j), name="c9")

        # 10. fuel level parameter to be bounded between 0 and L
        c10 = model.addConstrs((0 <= r[i] <= L for i in T), name="c10")

        # 11. Total fuel consumed by the UAV must be less than or equal to  𝐿  times the total number of refueling visits
        #c11 = model.addConstrs(((quicksum(f[i, j] * x[i, j, k] for i in N for j in N if i != j)) <=
        #                        L * (quicksum(x[d, i, k] for i in N for d in D if d != i)) for k in K), name="c11")

        # 12. Ensure that each robot is back at the start location before 𝑇𝑚𝑎𝑥
        c12 = model.addConstrs(
            (quicksum(c[i, j] * x[i, j, k] / self.vel for i in N for j in N if i != j) <= T_max
             for k in K), name="c12")

        '''capacity & flow constraints'''
        # 13. total capacity at start
        c13 = model.addConstrs(((quicksum((p[s, i, k] - p[i, s, k]) for i in N for s in S if i not in S)) ==
                                (quicksum(x[i, j, k] for i in T for j in N if i != j and i not in S)) for k in K), name="c13")

        # 14. capacity reduced by 1
        c14 = model.addConstrs(((quicksum((p[j, i, k] - p[i, j, k]) for j in N if i != j)) ==
                                (quicksum(x[i, j, k] for j in N if i != j)) for i in T for k in K if i not in S), name="c14")

        # 15. As the robot passes through refueling depots, though, this target capacity is prevented from changing
        c15 = model.addConstrs(((quicksum(
            (p[j, i, k] - p[i, j, k]) for j in N if j != i)) == 0 for i in D for k in K if i not in S), name="c15")

        # 16. target capacity for each robot does not exceed |𝑇|
        c16 = model.addConstrs((0 <= p[i, j, k] <= noOfTasks * x[i, j, k]
                                for i in N for j in N for k in K if i != j), name="c16")

        # Optimize
        #model.params.Heuristics = 0  # Do not use a heuristic solution
        #model.params.Cuts = 0  # Do not use cuts, except lazy constraints
        model.params.TimeLimit = 30
        model.optimize()

        return c, model




def main():

    '''Run TOPF with randomly generated input'''
    # Provide basic input
    noOfRobots = 4
    noOfTasks = 8
    noOfDepots = 2
    L = 200
    T_max = 500
    velocity = 1

    # randomly generated locations of tasks and robots
    K, T, D, S, E, T_loc, D_loc, N_loc, S_loc, E_loc, R = env.generate_test_instance_topf(noOfRobots, noOfTasks, noOfDepots)

    # Object of the planner
    milp = TOPF(velocity)
    # Optimize model
    c, plan = milp.new_planner(K, T, D, S, E, N_loc, noOfRobots, noOfTasks, L, T_max, R)

    # Plot the routes using plotly interactive GUI
    draw = Visualization_TOPF(K, T, D, S, T_loc, D_loc, S_loc, E_loc, c)
    # filename if the plot to be saved
    name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)
    # plot and save
    auto_open_flag = 1
    draw.save_plot_topf_milp(plan, name, auto_open_flag)



if __name__ == "__main__":
    main()
