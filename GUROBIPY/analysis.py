''' Model analysis class
    * Average computation time as a function of number of tasks and robots
    * Tasks dropped
    * ...

This file is meant to be run independently after running and recording experiments.
'''


# Importing existing python modules
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib import cm
import csv
import numpy as np
#import pandas as pd


''' Ananlysis class to infer trends in computation time'''
class Analysis_TOPF:
    def __init__(self, filename, location):
        '''
        Initialize
        :param filename: .csv filename
        :param location: path to the file
        '''
        self.filename = filename
        self.directory = location
        self.robots_range = []
        self.task_range = []
        self.depots_range = []
        self.L = []
        self.Tmax_range = []
        self.computations = []

        # Parse the .csv file
        csvFile = open(location+filename)
        data = list(csv.reader(csvFile))
        csvFile.close()
        # data = pd.read_table(location+filename, sep=" ")

        for row in data:
            row = list(map(float, row))
            #print(row[0])
            self.robots_range.append(row[0])
            self.task_range.append(row[1])
            self.depots_range.append(row[2])
            self.L.append(row[3])
            self.Tmax_range.append(row[4])
            self.computations.append(row[5])




    def plot_computation_time_graph_vs_KT(self):
        '''
        Plotting runtime vs robots, tasks
        :return: Plot
        '''

        # # Surface plot TODO: computations should be a matrix
        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # # Plot the surface.
        # surf = ax.plot_surface(np.array(self.robots_range), np.array(self.task_range), np.array(self.computations), cmap=cm.coolwarm, linewidth=0, antialiased=False)
        #
        # # Customize the z axis.
        # ax.zaxis.set_major_locator(LinearLocator(10))
        # ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
        #
        # # Managing x,y grid lines
        # ax.set_xticks(self.robots_range)
        # ax.set_yticks(self.task_range)
        # ax.set_zticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))
        #
        # # Label axes
        # ax.set_xlabel('robots')
        # ax.set_ylabel('tasks')
        # ax.set_zlabel('computation time')
        #
        # # Add a color bar which maps values to colors.
        # fig.colorbar(surf, shrink=0.5, aspect=5)

        # Graph
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        # Plot the graph
        ax.scatter(np.array(self.robots_range), np.array(self.task_range), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.robots_range)
        ax.set_yticks(self.task_range)
        #ax.set_zticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('robots')
        ax.set_ylabel('tasks')
        ax.set_zlabel('computation time')


        plt.show()

    def plot_computation_time_graph_vs_K(self):
        '''
        Plotting runtime vs robots
        :return: Plot
        '''

        print(np.array(self.robots_range))
        # print(self.computations)
        # Graph
        fig = plt.figure()
        ax = fig.gca()
        # Plot the graph
        plt.scatter(np.array(self.robots_range), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.robots_range)
        # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('robots')
        ax.set_ylabel('computation time')


        plt.show()


    def plot_computation_time_graph_vs_T(self):
        '''
        Plotting runtime vs tasks
        :return: Plot
        '''

        print(np.array(self.task_range))
        # print(self.computations)
        # Graph
        fig = plt.figure()
        ax = fig.gca()
        # Plot the graph
        plt.scatter(np.array(self.task_range), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.task_range)
        # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('tasks')
        ax.set_ylabel('computation time')


        plt.show()

    def plot_computation_time_graph_vs_T_boxplot(self):
        '''
        Plotting runtime vs tasks
        :return: Plot
        '''

        K = [3,4,5,6]
        task_temp3 = []
        computation3 = []
        for i in range(0,len(self.robots_range)):
            if(self.robots_range[i] == K[0]):
                task_temp3.append(self.task_range[i])
                computation3.append(self.computations[i])
        print(task_temp3)
        print(computation3)

        task_temp4 = []
        computation4 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[1]):
                task_temp4.append(self.task_range[i])
                computation4.append(self.computations[i])

        task_temp5 = []
        computation5 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[2]):
                task_temp5.append(self.task_range[i])
                computation5.append(self.computations[i])

        task_temp6 = []
        computation6 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[3]):
                task_temp6.append(self.task_range[i])
                computation6.append(self.computations[i])

        # Graph
        fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(4, 4))

        # Plot the graph
        axes[0, 0].scatter(np.array(task_temp3), np.array(computation3))
        axes[0, 1].scatter(np.array(task_temp4), np.array(computation4))
        axes[1, 0].scatter(np.array(task_temp5), np.array(computation5))
        axes[1, 1].scatter(np.array(task_temp6), np.array(computation6))

        # Managing x,y grid lines
        axes[0, 0].set_title('robots = 3')
        axes[0, 1].set_title('robots = 4')
        axes[1, 0].set_title('robots = 5')
        axes[1, 1].set_title('robots = 6')
        axes[0, 0].set_xticks(task_temp3)
        axes[0, 1].set_xticks(task_temp3)
        axes[1, 0].set_xticks(task_temp3)
        axes[1, 1].set_xticks(task_temp3)


        # # Label axes
        axes[0, 0].set_xlabel('tasks')
        axes[0, 0].set_ylabel('computation time')

        fig.tight_layout()
        plt.show()


    def plot_computation_time_graph_vs_D(self):
        '''
        Plotting runtime vs depots
        :return: Plot
        '''

        print(np.array(self.depots_range))
        # print(self.computations)
        # Graph
        fig = plt.figure()
        ax = fig.gca()
        # Plot the graph
        plt.scatter(np.array(self.depots_range), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.depots_range)
        # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('depots')
        ax.set_ylabel('computation time')


        plt.show()

    def plot_computation_time_graph_vs_TD(self):
        '''
        Plotting runtime vs nodes
        :return: Plot
        '''

        # print(self.computations)
        # Graph
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        # Plot the graph
        ax.scatter(np.array(self.task_range), np.array(self.depots_range), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.task_range)
        ax.set_yticks(self.depots_range)
        # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('tasks')
        ax.set_ylabel('depots')
        ax.set_zlabel('computation time')

        plt.show()



    def plot_computation_time_graph_vs_L(self):
        '''
        Plotting runtime vs fuel
        :return: Plot
        '''

        print(np.array(self.L))
        # print(self.computations)
        # Graph
        fig = plt.figure()
        ax = fig.gca()
        # Plot the graph
        plt.scatter(np.array(self.L), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.L)
        # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('fuel')
        ax.set_ylabel('computation time')

        plt.show()


    def plot_computation_time_graph_vs_TL(self):
        '''
        Plotting runtime vs
        :return: Plot
        '''

        print(np.array(self.L))
        # print(self.computations)
        # Graph
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        # Plot the graph
        ax.scatter(np.array(self.task_range), np.array(self.L), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.task_range)
        ax.set_yticks(self.L)
        # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('tasks')
        ax.set_ylabel('fuel')
        ax.set_zlabel('computation time')

        plt.show()



    def plot_computation_time_graph_vs_Tmax(self):
        '''
        Plotting runtime vs T_max
        :return: Plot
        '''

        print(np.array(self.Tmax_range))
        # print(self.computations)
        # Graph
        fig = plt.figure()
        ax = fig.gca()
        # Plot the graph
        plt.scatter(np.array(self.Tmax_range), np.array(self.computations))

        # Managing x,y grid lines
        ax.set_xticks(self.Tmax_range)
        # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('T_max')
        ax.set_ylabel('computation time')

        plt.show()


def main():
    print("Analysis of experiments performed")
    # .csv file and its location
    name = 'expt1.csv'
    dir = '../../../'

    # object
    inst = Analysis_TOPF(name, dir)

    # Plot TOPF experiment data
    # inst.plot_computation_time_graph_vs_K()
    #inst.plot_computation_time_graph_vs_T()
    inst.plot_computation_time_graph_vs_L()
    #inst.plot_computation_time_graph_vs_T_boxplot()
    # inst.plot_computation_time_graph_vs_D()
    # inst.plot_computation_time_graph_vs_TD()
    # inst.plot_computation_time_graph_vs_Tmax()
    # inst.plot_computation_time_graph_vs_KT()



if __name__ == "__main__":
        main()



