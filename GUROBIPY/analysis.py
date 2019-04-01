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


    def plot_computation_time_graph_vs_T(self):
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
    #inst.plot_computation_time_graph_vs_K()
    #inst.plot_computation_time_graph_vs_T()
    #inst.plot_computation_time_graph_vs_D()
    inst.plot_computation_time_graph_vs_L()
    inst.plot_computation_time_graph_vs_T()
    # inst.plot_computation_time_graph_vs_KT()



if __name__ == "__main__":
        main()



