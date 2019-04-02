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

# Importing required project modules
import environment as env
import topf
#import toptw
from visualization import Visualization_TOPF
from collection import save_topf_data, save_toptw_data, save_topf_data_heuristic
from heuristics import Heuristics_TOPF

def single_run_topf_random_input(noOfRobots, noOfTasks, noOfDepots, L, T_max, velocity, auto_open_flag=0):
    '''
    Finds the optimized routes for randomly generated input
    :param noOfRobots: number of robots
    :param noOfTasks: number of tasks/targets
    :param noOfDepots: number of depots
    :param L: maximum fuel of the robot
    :param T_max: total flight time
    :param velocity: velocity of the robots
    :param auto_open_flag: 0 to disable plot pop-up
    :return: Nothing, just plots the interactive route for each robot
    '''

    # randomly generated locations of tasks and robots
    K, T, D, S, T_loc, D_loc, N_loc = env.generate_test_instance_topf(noOfRobots, noOfTasks, noOfDepots)

    # Object of the planner
    milp = topf.TOPF(velocity)
    # Optimize model
    c, plan = milp.planner(K, T, D, S, N_loc, noOfRobots, noOfTasks, L, T_max)

    # Plot the routes using plotly interactive GUI
    draw = Visualization_TOPF(K, T, D, S, T_loc, D_loc, c)
    # filename if the plot to be saved
    name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)
    # plot and save
    routes = draw.save_plot_topf_milp(pplan, name, auto_open_flag)
    # Print the planned schedule
    print(routes)


def multi_run_topf_random_input(robots_range, node_range, L_range, Tmax_range, velocity, expt_name, planner_flag, auto_open_flag=0):
    '''
    Multiple runs for TOPF experiments with randomly generated input
    :param robots_range: list of number of robots
    :param task_range: list of tasks
    :param depot_range: list of depots
    :param L_range: list of fuel of the robot
    :param Tmax_range: list of total flight time
    :param velocity: velocity of the robots
    :param expt_name: name of .csv file
    :param planner_flag: 1 for planning without capacity constraints
    :param auto_open_flag: 0 to disable plot pop-up
    :return: Nothing, just plots (.html) the interactive route for each robot and saves data (.csv)
    '''
    input_data = env.generate_input_combinations(robots_range, node_range, L_range, Tmax_range)
    for row in input_data:
        [noOfRobots, noOfTasks, noOfDepots, L, T_max] = row
        noOfRobots = int(noOfRobots)
        noOfTasks = int(noOfTasks)
        noOfDepots = int(noOfDepots)
        T_max = int(T_max)
        print("===========================================================")
        print(noOfRobots, noOfTasks, noOfDepots, L, T_max)

        # itr to track number of re-runs
        count = 0
        # to re-run the randomly generated input if the model turns out to be infeasible
        while(True):

            # randomly generated locations of tasks and robots
            K, T, D, S, T_loc, D_loc, N_loc = env.generate_test_instance_topf(int(noOfRobots), int(noOfTasks), int(noOfDepots))

            # Object of the planner
            milp = topf.TOPF(velocity)
            # Optimize model
            if(planner_flag == 1):
                c, plan = milp.planner(K, T, D, S, N_loc, noOfRobots, noOfTasks, L, T_max)
            else:
                c, plan = milp.planner_with_flow(K, T, D, S, N_loc, noOfRobots, noOfTasks, L, T_max)

            # Actual runtime
            # get out of the loop if runtime (model) is feasible
            #runtime = plan.Runtime

            if (plan.status == GRB.Status.INF_OR_UNBD or plan.status == GRB.Status.INFEASIBLE \
                    or plan.status == GRB.Status.UNBOUNDED):
                if(count < 5):
                    count = count+1
                    continue
                else:
                    break
            elif (plan.status == GRB.Status.OPTIMAL):

                # Plot the routes using plotly interactive GUI
                draw = Visualization_TOPF(K, T, D, S, T_loc, D_loc, c)
                # filename if the plot to be saved
                if (planner_flag == 1):
                    name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)
                else:
                    name = 'Fplot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)

                # plot and save
                routes = draw.save_plot_topf_milp(plan, name, auto_open_flag)
                print(routes)

                # For data collection, generates .csv file
                save_topf_data(plan, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name)





def run_topf_with_test_instances(downloaded_instances, directory, expt_name, auto_open_flag=0):
    '''
    Finds the optimized routes for standard test instances
    :param downloaded_instances: 0 is C-mdvrp
    :param directory: path where the extracted folder is stored
    :param expt_name: name of .csv file
    :param auto_open_flag: 0 to disable plot pop-up
    :return: Nothing, just plots (.html) the interactive route for each robot and saves data (.csv)
    '''
    # TODO: add downloaded_instances switch case

    # For all files (test instances) in the directory
    for filename in os.listdir(directory):
        print("===========================================================")
        print(filename)
        # Read the files one by one and parse it to the variables accordingly
        noOfRobots, noOfTasks, noOfDepots, L, T_max, K, T, D, S, T_loc, D_loc, N_loc = env.get_input_data_topf(
            directory + filename)

        # Velocity of the robots assumed to be constant and same for each robot
        # TODO: manipulate velocity of robots
        velocity = 1

        # Object of the planner
        milp = topf.TOPF(velocity)
        # Optimize model
        c, plan = milp.planner(K, T, D, S, N_loc, noOfRobots, noOfTasks, L, T_max)

        # Plot the routes using plotly interactive GUI
        draw = Visualization_TOPF(K, T, D, S, T_loc, D_loc, c)
        # filename if the plot to be saved
        name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)
        # plot and save
        routes = draw.save_plot_topf_milp(plan, name, auto_open_flag)
        print(routes)

        # For data collection, generates .csv file
        save_topf_data(plan, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name)


def single_run_heuristics(noOfRobots, noOfTasks, noOfDepots, L, T_max, velocity):
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
    name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)
    # plot and save
    auto_open_flag = 0
    draw.save_plot_topf_heuristic(arcsInOrderHeuristic, name, auto_open_flag)



def multi_run_heuristics( robots_range, node_range, L_range, Tmax_range, velocity, expt_name):
    input_data = env.generate_input_combinations(robots_range, node_range, L_range, Tmax_range)
    for row in input_data:
        [noOfRobots, noOfTasks, noOfDepots, L, T_max] = row
        noOfRobots = int(noOfRobots)
        noOfTasks = int(noOfTasks)
        noOfDepots = int(noOfDepots)
        T_max = int(T_max)
        print("===========================================================")
        print(noOfRobots, noOfTasks, noOfDepots, L, T_max)

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
        name = 'plot' + str(noOfRobots) + '_' + str(noOfTasks) + '_' + str(noOfDepots)
        # plot and save
        auto_open_flag = 0
        draw.save_plot_topf_heuristic(arcsInOrderHeuristic, name, auto_open_flag)

        # For data collection, generates .csv file
        save_topf_data_heuristic(seed, cost, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name)


def single_run_toptw_random_input(noOfWorkerRobots, noOfTasks, noOfStartNodes, maxTaskDuration, T_max, maxTimeInterval, velocity, auto_open_flag=0):
    '''
    TOPTW for randomly generated input
    :param noOfWorkerRobots: number of robots
    :param noOfTasks: number of tasks
    :param noOfStartNodes: number of start nodes
    :param maxTaskDuration: max possible task duration
    :param T_max: total flight time
    :param maxTimeInterval: time interval between start and end times of the tasks
    :param velocity: velocity of the robots assumed to be constant
    :param auto_open_flag: 0 to disable plot pop-up
    :return: Nothing, just plots (.html) the interactive route for each robot
    '''
    # randomly generated locations of tasks and robots
    W, S, T, E, N_loc, Q, O, C, D = env.generate_test_instance_toptw(noOfWorkerRobots, noOfTasks, noOfStartNodes,
                                                             maxTaskDuration, T_max, maxTimeInterval)
    # Object of the planner
    milp = toptw.TOPTW(velocity)
    # Optimize!!!
    plan = milp.planner(W, S, T, E, N_loc, noOfWorkerRobots, Q, O, C, D, T_max)




def multi_run_toptw_random_input(robots_range, task_range, Tmax_range, velocity, expt_name, noOfStartNodes, maxTaskDuration, maxTimeInterval):
    '''
    Multiple runs for TOPTW experiments with randomly generated input
    ::param robots_range: list of number of robots
    :param task_range: list of tasks
    :param Tmax_range: list of total flight time
    :param velocity: velocity of the robots
    :param expt_name: name of .csv file
    :param auto_open_flag: 0 to disable plot pop-up
    :return: Nothing, just saves data (.csv)
    '''
    for T_max, noOfWorkerRobots, noOfTasks in itertools.product(Tmax_range, robots_range, task_range):

        print("===========================================================")
        print(noOfWorkerRobots, noOfTasks, T_max)

        # to re-run the randomly generated input if the model turns out to be infeasible
        while(True):

            # randomly generated locations of tasks and robots
            W, S, T, E, N_loc, Q, O, C, D = env.generate_test_instance_toptw(noOfWorkerRobots, noOfTasks, noOfStartNodes, maxTaskDuration, T_max, maxTimeInterval)

            # Object of the planner
            milp=toptw.TOPTW(velocity)
            # Optimize!!!
            plan = milp.planner(W, S, T, E, N_loc, noOfWorkerRobots, Q, O, C, D, T_max)
            # Actual runtime
            # get out of the loop if runtime (model) is feasible
            #runtime = plan.Runtime

            if(plan.status == GRB.Status.INF_OR_UNBD or plan.status == GRB.Status.INFEASIBLE\
                    or plan.status == GRB.Status.UNBOUNDED):
                continue
            elif(plan.status == GRB.Status.OPTIMAL):
                break



        # For data collection, generates .csv file
        save_toptw_data(plan, noOfWorkerRobots, noOfTasks, T_max, expt_name)




def main():
    '''
    Main function
    '''

    ###################################################################################################################

    '''Run TOPF MILP with randomly generated input'''
    # # Provide basic input
    # noOfRobots = 3
    # noOfTasks = 8
    # noOfDepots = 2
    # L = 200
    # T_max = 500
    # velocity = 1
    # # Generate input -> Plan -> Plot & Save image (.html)
    # single_run_topf_random_input(noOfRobots, noOfTasks, noOfDepots, L, T_max, velocity)

    ###################################################################################################################

    '''Experiments: TOPF MILP with randomly generated input '''
    # # change the name as per experiment
    # expt_name = 'topf_experiment_5R'
    # # Provide basic input for multiple runs
    # robots_range = [2]  #[2, 3, 4, 5, 6, 7, 8, 9, 10]
    # # task_range = [6,7,8]
    # # depot_range = [1,2,3]
    # node_range = [5, 10, 15, 20, 25, 30]
    # L_range = [100 * np.sqrt(2), 100 * 2 * np.sqrt(2)]  #for arena size 100 x 100
    # Tmax_range = [400]
    # velocity = 1
    #
    # # Generate input -> Plan -> Plot & Save image (.html) -> Save computational data (.csv)
    # multi_run_topf_random_input(robots_range, node_range, L_range, Tmax_range, velocity, expt_name, planner_flag=1)
    #
    # expt_name = 'topf_experiment_5R_with_flow'
    # multi_run_topf_random_input(robots_range, node_range, L_range, Tmax_range, velocity, expt_name, planner_flag=2)



    ###################################################################################################################

    '''Run TOPF MILP with C-mdvrp test instances'''
    # expt_name = 'topf_cmdvrp13'
    # # Select which instance set
    # downloaded_instances = 0
    # # Location of the saved instances
    # directory = 'C-mdvrp/'
    # # Read input -> Plan -> Plot & Save image (.html) -> Save computational data (.csv)
    # run_topf_with_test_instances(downloaded_instances, directory, expt_name, auto_open_flag=0)


    ###################################################################################################################









    ###################################################################################################################

    '''Run TOPF heuristic with randomly generated input'''
    # noOfRobots = 3
    # noOfTasks = 8
    # noOfDepots = 2
    # L = 200
    # T_max = 500
    # velocity = 1
    #
    # single_run_heuristics(noOfRobots, noOfTasks, noOfDepots, L, T_max, velocity)


    ###################################################################################################################

    '''Experiments: TOPF heuristic with randomly generated input '''
    # change the name as per experiment
    expt_name = 'topf_experiment_2RH'
    # Provide basic input for multiple runs
    robots_range = [2]  # [2, 3, 4, 5, 6, 7, 8, 9, 10]
    # task_range = [6,7,8]
    # depot_range = [1,2,3]
    node_range = [5, 10, 15, 20, 25, 30]
    L_range = [100 * np.sqrt(2), 100 * 2 * np.sqrt(2)]  # for arena size 100 x 100
    Tmax_range = [400]
    velocity = 1

    # Generate input -> Plan -> Plot & Save image (.html) -> Save computational data (.csv)
    multi_run_heuristics(robots_range, node_range, L_range, Tmax_range, velocity, expt_name)


    ###################################################################################################################






    ###################################################################################################################

    '''Run TOPTW with randomly generated input'''
    # # Provide basic input
    # noOfTasks = 8
    # noOfWorkerRobots = 4
    # noOfStartNodes = 1
    # maxTimeInterval = 50
    # maxStartTime = 400 - maxTimeInterval
    # maxTaskDuration = 10
    # velocity = 1
    # T_max = 1000
    # # Read input -> Plan -> Save computational data (.csv)
    # single_run_toptw_random_input(noOfWorkerRobots, noOfTasks, noOfStartNodes, maxTaskDuration, T_max, maxTimeInterval, velocity)

    ###################################################################################################################

    '''Experiments: TOPTW with randomly generated input '''
    # expt_name = 'toptw_experiment1'
    # # Provide basic input for multiple runs
    # robots_range = [2]
    # task_range = [8]
    # Tmax_range = [1000]
    # noOfStartNodes = 1
    # maxTimeInterval = 50
    # maxStartTime = 400 - maxTimeInterval
    # maxTaskDuration = 10
    # velocity = 1
    # # Generate input -> Plan -> Save computational data (.csv)
    # multi_run_toptw_random_input(robots_range, task_range, Tmax_range, velocity, expt_name, noOfStartNodes, maxTaskDuration, maxTimeInterval)

    ###################################################################################################################


if __name__ == "__main__":
     main()



