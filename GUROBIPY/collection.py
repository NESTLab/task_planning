'''
Saves experimental data in comma separated format (.csv)
'''

# Importing existing python modules
import csv



def save_topf_data(plan, noOfRobots, noOfTasks, noOfDepots, L, T_max, expt_name):
    runtime = plan.Runtime
    # Save data
    row = [noOfRobots, noOfTasks, noOfDepots, L, T_max, runtime]

    with open(expt_name+'.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)

    csvFile.close()



def save_toptw_data(plan, noOfWorkerRobots, noOfTasks, T_max, expt_name):
    runtime = plan.Runtime
    # Save data
    row = [noOfWorkerRobots, noOfTasks, T_max, runtime]

    with open(expt_name+'.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)

    csvFile.close()


