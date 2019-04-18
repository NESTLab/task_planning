'''
Saves experimental data in comma separated format (.csv)
'''

# Importing existing python modules
import csv



def save_topf_data(plan, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name, env_seed=0):
    runtime = plan.Runtime
    quality = plan.ObjVal
    # Save data
    row = [noOfRobots, noOfTasks, noOfDepots, L, T_max, runtime, quality, env_seed]

    with open(expt_name+'.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)
    csvFile.close()

    dataForPlots = [noOfTasks, quality, env_seed]
    with open('data_'+expt_name+'.csv', 'a') as csvFile2:
        writer = csv.writer(csvFile2)
        writer.writerow(dataForPlots)
    csvFile2.close()



def save_topf_data_heuristic(seed,cost, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name):

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
