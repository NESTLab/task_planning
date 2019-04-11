"""
This file aims to provide the required data (locations, task attributes, etc), depending on the formulation,
either generated randomly or extracted from input files.

It accepts input text (.txt) files.

This script requires that
    *'random'
    * 'os'
be installed within the Python environment you are running this script in.

This file can also be imported as a module and contains the following functions:
    * generate_test_instance - generates the required input randomly
    * get_input_data - returns the standard test instances
    * main - the main function of the script

"""

# Importing existing python modules
import itertools
import random as rnd
import sys
import os
import numpy as np


def generate_input_combinations(robots_range, node_range, L_range, Tmax_range):

    input_object = itertools.product(robots_range, node_range, L_range, Tmax_range)
    input_data = []
    for r, n, l, time in input_object:
        task_set = [i for i in range(1, n)]
        depot_set = [i for i in range(1, n)]
        all_combinations = itertools.product(task_set, depot_set)
        combinations = []
        for t,d in all_combinations:
            if(t+d == n):
                combinations.append([t,d])
        #print(combinations)
        for t,d in combinations:
            row = list([r, t, d, l, time])
            input_data.append(row)

    input_data = np.array(input_data)
    #print(input_data)

    # unique_input_data = np.unique(input_data, axis=0)
    # print(unique_input_data)

    return input_data



def generate_test_instance_topf(noOfRobots, noOfTasks, noOfDepots):
    '''
    Generating locations randomly in a 100 x 100 arena
    :param noOfRobots: number of robots
    :param noOfTasks: number of tasks/targets
    :param noOfDepots: number of depots = refueling stations
    :return:K: set of robots
            T: set of tasks
            D: set of depots
            S: set of robot starting locations
            T_loc: set of task locations
            D_loc: set of depot locations
            N_loc: set of nodes (locations)

    '''
    thisSeed = rnd.randrange(sys.maxsize)
    rng = rnd.Random(thisSeed)
    rnd.seed(thisSeed)

    # creating sets
    T = ["T" + str(i) for i in range(noOfTasks)]
    K = ["K" + str(i) for i in range(noOfRobots)]
    D = ["D" + str(i) for i in range(noOfDepots)]
    S = ['S0']
    E = ['E0']

    # randomly generate location in 100 x 100 arena
    T_loc = {task: (100 * rnd.random(), 100 * rnd.random()) for task in T}
    D_loc = {loc: (100 * rnd.random(), 100 * rnd.random()) for loc in D}

    S_loc = {loc: D_loc['D0'] for loc in S}
    E_loc = {loc: D_loc['D0'] for loc in E}
    N = T + D + S + E

    # set of nodes
    N_loc = {**T_loc, **D_loc, **S_loc, **E_loc}

    # reward
    R = {task: rnd.randint(1, 100) for task in T}

    return K, T, D, S, E, T_loc, D_loc, N_loc, S_loc, E_loc, R, thisSeed

def get_input_data_topf(filename):
    '''
    Parse standard test instances into usable input
    :param filename: name of the input file
    :return:noOfRobots: total number of robots
            noOfTasks: total number of tasks
            noOfDepots: total number of depots
            L: max fuel of each robot
            T_max: total flight time
            K: set of robots
            T: set of tasks
            D: set of depots
            S: set of robot starting locations
            T_loc: set of task locations
            D_loc: set of depot locations
            N_loc: set of nodes (locations)
    '''
    # open the inout file
    f = open(filename)
    # reading the first line
    first_line = f.readline()
    #print(first_line)
    separate = first_line.split(" ")
    separate = list(map(int, separate))
    #print(separate)

    # check if its C-mdvrp file (first number == 2)
    if(separate[0] == 2):
        # parsing first line
        noOfRobots = separate[1]
        noOfTasks = separate[2]
        noOfDepots = separate[3]
        #print(noOfRobots, noOfTasks, noOfDepots)
        # creating sets
        T = ["T" + str(i) for i in range(noOfTasks)]
        K = ["K" + str(i) for i in range(noOfRobots)]
        D = ["D" + str(i) for i in range(noOfDepots)]
        #S = ['D0']
        S = ['S0']
        E = ['E0']

        # parsing second line
        second_line = f.readline()
        separate = second_line.split(" ")
        separate = list(map(int, separate))
        #print(separate)
        L = separate[1]


        # Leave out not-applicable info
        for i in range(1,noOfDepots):
            extra = f.readline()

        # Parsing x,y coordinates of tasks
        T_loc = {}
        D_loc = {}
        for i in range(0, noOfTasks):
            task_lines = f.readline()
            #print(task_lines)
            coordinates = task_lines.split(" ")
            while ("" in coordinates):
                coordinates.remove("")
            coordinates = list(map(int, coordinates))
            #print(coordinates)
            T_loc[T[i]] = coordinates[1:3]

        # Parsing x,y coordinates of depots
        for i in range(0, noOfDepots):
            depot_lines = f.readline()
            #print(depot_lines)
            coordinates = depot_lines.split(" ")
            while ("" in coordinates):
                coordinates.remove("")
            coordinates = list(map(int, coordinates))
            #print(coordinates)
            D_loc[D[i]] = coordinates[1:3]

    else:
        # File check failed
        print("Input file not recognized")
        return
    S_loc = {loc: D_loc['D0'] for loc in S}
    E_loc = {loc: D_loc['D0'] for loc in E}
    N = T + D + S + E

    N_loc = {**T_loc, **D_loc}

    N_loc = {**T_loc, **D_loc, **S_loc, **E_loc}

    # reward
    R = {task: 1 for task in T}

    # Add T_max
    # TODO: T_max = constant
    T_max = 1000

    return noOfRobots, noOfTasks, noOfDepots, L, T_max, K, T, D, S, E, T_loc, D_loc, N_loc, S_loc, E_loc, R



def generate_test_instance_toptw(noOfWorkerRobots, noOfTasks, noOfStartNodes, maxTaskDuration, maxStartTime, maxTimeInterval):
    '''
    Generating locations randomly in a 100 x 100 arena
    :param noOfWorkerRobots: number of robots
    :param noOfTasks: number of tasks
    :param noOfStartNodes: number of robot start nodes
    :param maxTaskDuration: maximum approximate task duration
    :param maxStartTime: maximum time before which the start execution should start
    :param maxTimeInterval: time interval between start and end times of the tasks
    :return: W: set of robots
             S: set of robot start locations
             T: set of tasks
             E: set of end nodes
             N_loc: set of node locations
             Q: set of quotas for each task
             O: set of start times
             C: set of task max start times
             D: task duration
    '''
    thisSeed = rnd.randrange(sys.maxsize)
    rng = rnd.Random(thisSeed)
    rnd.seed(thisSeed)

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

    return W, S, T, E, S_loc, E_loc, T_loc, N_loc, Q, O, C, D, thisSeed




def main():

    '''Test the feasibility of C-mdvrp instances'''
    directory = 'C-mdvrp/'
    # for filename in os.listdir(directory):
    #     print("===========================================================")
    #     print(filename)
    #     noOfRobots, noOfTasks, noOfDepots, L, T_max, K, T, D, S, T_loc, D_loc, N_loc = get_input_data_topf(directory + filename)
    #     print(noOfRobots, noOfTasks, noOfDepots, L, T_max)
    filename = 'p12'
    noOfRobots, noOfTasks, noOfDepots, L, T_max, K, T, D, S, T_loc, D_loc, N_loc = get_input_data_topf(
        directory + filename)
    print(noOfRobots, noOfTasks, noOfDepots, L, T_max)

    '''Test input combinations'''
    # robots_range = [2]# 3, 4, 5, 6, 7, 8, 9, 10]
    # node_range = [4, 5, 9, 10, 14, 15, 19, 20, 24, 25, 29, 30]
    # L_range = [100 * np.sqrt(2), 100 * 2 * np.sqrt(2)]  # for arena size 100 x 100
    # Tmax_range = [400]
    # print(generate_input_combinations(robots_range, node_range, L_range, Tmax_range))


if __name__ == "__main__":
    main()
