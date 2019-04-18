'''
Main file to run optimization, test cases, data collection and visualization for task planning of multi-robot systems.

It accepts input text (.txt) files.

This script requires that
    *'gurobipy'
    *'os'
    *'itertools'
be installed within the Python environment you are running this script in.

This file can also be imported as a module and contains the following functions:

    * generate_test_instance - generates the required input
    * get_input_data_topf - returns the required input
    * main - the main function of the script
'''


# Importing existing python modules
import os
import itertools
import numpy as np
from gurobipy import *
import csv
import random as rnd
import sys

# Importing required project modules
import environment as env
import topf
import toptw
from visualization import Visualization_TOPF, Visualization_TOPTW
from collection import save_topf_data, save_toptw_data, save_topf_data_heuristic
from heuristics import Heuristics_TOPF


def main():
    '''
    Main function
    '''
    # Basic Input
    # csvFile = open('input_combinations.csv')
    # input_data = list(csv.reader(csvFile))
    # csvFile.close()
    robots_range = [2]
    # , 15, 20]#, 25, 30, 35, 40, 45, 50]
    node_range = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    L_range = [75]
    Tmax_range = [200]
    input_data = env.generate_input_combinations(
        robots_range, node_range, L_range, Tmax_range)

    thisSeed = rnd.randrange(sys.maxsize)
    rnd.seed(thisSeed)
    print("Seed:", thisSeed)

    velocity = 1

    ###################################################################################################################
    expt_name = 'topf_experiment'

    for row in input_data:
        [noOfRobots, noOfTasks, noOfDepots, L, T_max] = row
        noOfRobots = int(noOfRobots)
        noOfTasks = int(noOfTasks)
        noOfDepots = int(noOfDepots)
        L = float(L)
        T_max = float(T_max)
        print("===========================================================")
        print(noOfRobots, noOfTasks, noOfDepots, L, T_max)

        # itr to track number of re-runs
        retry_count = 0
        # to re-run the randomly generated input if the model turns out to be infeasible
        while(True):
            print("-------------------TOPF MILP Solution---------------------------")

            # randomly generated locations of tasks and robots
            K, T, D, S, E, T_loc, D_loc, N_loc, S_loc, E_loc, R = env.generate_test_instance_topf(noOfRobots, noOfTasks,
                                                                                                  noOfDepots, thisSeed)
            # Object of the planner
            milp_topf = topf.TOPF(velocity)
            # Optimize model
            c, plan = milp_topf.new_planner(
                K, T, D, S, E, N_loc, noOfRobots, noOfTasks, L, T_max, R)

            # get out of the loop if runtime (model) is feasible
            if (plan.status == GRB.Status.INF_OR_UNBD or plan.status == GRB.Status.INFEASIBLE
                    or plan.status == GRB.Status.UNBOUNDED or plan.ObjVal <= 0):
                if (retry_count < 15):
                    retry_count = retry_count + 1
                    continue
                else:
                    break
            elif (plan.status == GRB.Status.OPTIMAL):
                # Plot the routes using plotly interactive GUI
                draw = Visualization_TOPF(
                    K, T, D, S, R, T_loc, D_loc, S_loc, E_loc, c, L, T_max)
                # filename if the plot to be saved
                name = 'TOPF_' + str(noOfRobots) + '_' + \
                    str(noOfTasks) + '_' + str(noOfDepots)

                # plot and save
                routes = draw.save_plot_topf_milp(plan, name, 0)
                print(routes)
                print("Quality:", plan.ObjVal)
                # For data collection, generates .csv file
                save_topf_data(plan, noOfRobots, noOfTasks,
                               noOfDepots, L, T_max, expt_name+'_milp', thisSeed)

            print("-------------TOPF MINIMAX MILP Solution--------------------")

            c, plan = milp_topf.new_planner_minmax(
                K, T, D, S, E, N_loc, noOfRobots, noOfTasks, L, T_max, R)

           # get out of the loop if runtime (model) is feasible
            if (plan.status == GRB.Status.INF_OR_UNBD or plan.status == GRB.Status.INFEASIBLE
                    or plan.status == GRB.Status.UNBOUNDED or plan.ObjVal < 0):
                if (retry_count < 15):
                    retry_count = retry_count + 1
                    print('TOPF MINMAX Solution Unbounded or Infeasible or ObjVal < 0')
                    continue
                else:
                    break
            elif (plan.status == GRB.Status.OPTIMAL):
                retry_count = 0
                # Plot the routes using plotly interactive GUI
                draw = Visualization_TOPF(
                    K, T, D, S, R, T_loc, D_loc, S_loc, E_loc, c, L, T_max)
                # filename if the plot to be saved
                name = 'TOPF_' + str(noOfRobots) + '_' + \
                    str(noOfTasks) + '_' + str(noOfDepots) + '_MM'

                # plot and save
                routes = draw.save_plot_topf_milp(plan, name, 0)
                print(routes)
                print("Quality:", plan.ObjVal)
                # For data collection, generates .csv file
                save_topf_data(plan, noOfRobots, noOfTasks,
                               noOfDepots, L, T_max, expt_name+'_milpMM', thisSeed)
                '''
                print("-----------------Heuristic Approach------------------------")
                S = ['D0']
                E = ['D0']
                # Run heuristics for the same problem instance
                inst = Heuristics_TOPF(K, T, D, S, T_loc, D_loc, N_loc, L, T_max, velocity, thisSeed)
                finalHeuristicSolution, c, seed, cost = inst.ILS()

                arcsInOrderHeuristic = {}
                for k in K:
                    arcsInOrderHeuristic[k] = inst.pathArcRepresentation(finalHeuristicSolution[k])
                print(arcsInOrderHeuristic)
                print("Quality:", cost)
                # Plot the routes using plotly interactive GUI
                draw = Visualization_TOPF(K, T, D, S, R, T_loc, D_loc, S_loc, E_loc, c)
                # filename if the plot to be saved
                name = 'Hplot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)
                # plot and save
                auto_open_flag = 0
                draw.save_plot_topf_heuristic(arcsInOrderHeuristic, name, auto_open_flag)

                # For data collection, generates .csv file
                save_topf_data_heuristic(seed, cost, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name+'_heuristic')
                '''
                break

    ###################################################################################################################
    '''
    maxTimeInterval = 50
    maxTaskDuration = 10
    noOfStartNodes = 1

    expt_name = 'toptw_experiment'

    for row in input_data:
        [noOfRobots, noOfTasks, NA, NA, T_max] = row
        noOfRobots = int(noOfRobots)
        noOfTasks = int(noOfTasks)
        T_max = float(T_max)
        print("===========================================================")
        print(noOfRobots, noOfTasks, T_max)

        print("-------------------TOPTW MILP Solution---------------------------")
        # itr to track number of re-runs
        retry_count = 0
        # to re-run the randomly generated input if the model turns out to be infeasible
        while (True):
            # randomly generated locations of tasks and robots
            W, S, T, E, S_loc, E_loc, T_loc, N_loc, Q, O, C, D, R, A = env.generate_test_instance_toptw(
                noOfRobots, noOfTasks, noOfStartNodes, maxTaskDuration, T_max, maxTimeInterval, thisSeed)
            # Object of the planner
            milp_toptw = toptw.TOPTW(velocity)
            # Optimize!!!
            c, plan = milp_toptw.planner(W, S, T, E, N_loc, noOfRobots, Q, O, C, D, T_max, R)

            # get out of the loop if runtime (model) is feasible
            if (plan.status == GRB.Status.INF_OR_UNBD or plan.status == GRB.Status.INFEASIBLE
                    or plan.status == GRB.Status.UNBOUNDED or plan.ObjVal <= 0):
                if (retry_count < 50):
                    retry_count = retry_count + 1
                    continue
                else:
                    break
            elif (plan.status == GRB.Status.OPTIMAL):

                # Plot the routes using plotly interactive GUI
                draw = Visualization_TOPTW(plan, W, T, D, S, E, S_loc, E_loc, T_loc, c, A, Q, R)
                # filename if the plot to be saved
                name = 'Tplot' + str(noOfRobots) + '_' + str(noOfTasks)

                # plot and save
                routes = draw.save_plot_toptw(plan, name, 0)
                print(routes)

                # For data collection, generates .csv file
                save_toptw_data(plan, noOfRobots, noOfTasks, T_max, expt_name, thisSeed)

                break
    '''


if __name__ == "__main__":
    main()
