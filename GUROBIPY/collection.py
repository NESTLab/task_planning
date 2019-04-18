'''
Saves experimental data in comma separated format (.csv)
'''

# Importing existing python modules
import csv
import os.path


def save_topf_data(plan, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name, env_seed=0):
    runtime = plan.Runtime
    quality = plan.ObjVal
    MIPGap = plan.MIPGap
    # Save data
    row = [noOfRobots, noOfTasks, noOfDepots,
           L, T_max, runtime, quality, env_seed]

    with open(expt_name+'.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)
    csvFile.close()
    curr_exp = 'TOPF_{}_{}_{}'.format(noOfRobots, noOfTasks, noOfDepots)
    if 'MM' in expt_name:
        curr_exp += '_MM'
    # Add the iteration number coming from bash
    try:
        curr_exp += '_iter'+str(sys.argv[1])
    except IndexError:
        pass

    dataForPlots = [noOfTasks, quality, MIPGap, env_seed, curr_exp]
    if not os.path.isfile('data_'+expt_name+'.csv'):
        with open('data_'+expt_name+'.csv', 'a') as csvFile2:
            writer = csv.writer(csvFile2)
            writer.writerow(['noOfTasks', 'ObjVal', 'MIPGap',
                             ' env_seed', 'expt_name'])
        csvFile2.close()

    with open('data_'+expt_name+'.csv', 'a') as csvFile2:
        writer = csv.writer(csvFile2)
        writer.writerow(dataForPlots)
    csvFile2.close()


def save_topf_data_heuristic(seed, cost, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name):

    # Save data
    row = [noOfRobots, noOfTasks, noOfDepots, L, T_max, cost, seed]

    with open(expt_name+'.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)

    csvFile.close()


def save_toptw_data(plan, noOfWorkerRobots, noOfTasks, T_max, expt_name, seed):
    runtime = plan.Runtime
    quality = plan.ObjVal
    # Save data
    row = [noOfWorkerRobots, noOfTasks, T_max, runtime, quality, seed]

    with open(expt_name+'.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)

    csvFile.close()
