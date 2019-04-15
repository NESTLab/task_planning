''' Model analysis class
    * Average computation time as a function of number of tasks and robots
    * Tasks dropped
    * ...

This file is meant to be run independently after running and recording experiments.
'''


# Importing existing python modules
import plotly.offline as py
import plotly.tools as tls
import plotly.graph_objs as go
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from matplotlib import pylab
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.ticker import LinearLocator, FormatStrFormatter
# from matplotlib import cm
import csv
import numpy as np
#import pandas as pd


''' Ananlysis class to infer trends in computation time'''
class Analysis_TOPF:
    def __init__(self):
        '''
        Initialize
        :param filename: .csv filename
        :param location: path to the file
        '''
        self.robots_range = []
        self.task_range = []
        self.depots_range = []
        self.L = []
        self.Tmax_range = []
        self.computations = []
        self.quality = []

    def readfile(self, filename, location):
        # Parse the .csv file
        csvFile = open(location+filename)
        data = list(csv.reader(csvFile))
        csvFile.close()

        for row in data:
            row = list(map(float, row))
            #print(row[0])
            self.robots_range.append(row[0])
            self.task_range.append(row[1])
            self.depots_range.append(row[2])
            self.L.append(row[3])
            self.Tmax_range.append(row[4])
            self.computations.append(row[5])
            self.quality.append(row[6])

    def readfileH(self, filename, location):
        # Parse the .csv file
        csvFile = open(location+filename)
        data = list(csv.reader(csvFile))
        csvFile.close()

        for row in data:
            row = list(map(float, row))
            #print(row[0])
            self.robots_range.append(row[0])
            self.task_range.append(row[1])
            self.depots_range.append(row[2])
            self.L.append(row[3])
            self.Tmax_range.append(row[4])
            self.quality.append(row[5])

    def separate_nodes(self, nodes):
        length = len(self.task_range)
        node_range = []
        quality = []
        computations = []
        separate_task_range = []
        separate_depot_range = []

        for i in range(0, length):
           if (self.task_range[i] + self.depots_range[i] == nodes):
               quality.append(self.quality[i])
               node_range.append(nodes)
               computations.append(self.computations[i])
               separate_task_range.append(self.task_range[i])
               separate_depot_range.append(self.depots_range[i])

        return node_range, quality, computations, separate_task_range, separate_depot_range


    def separate_nodesH(self, nodes):
        length = len(self.task_range)
        node_range = []
        quality = []
        separate_task_range = []

        for i in range(0, length):
            if (self.task_range[i] + self.depots_range[i] == nodes):
                 quality.append(self.quality[i])
                 node_range.append(nodes)
                 separate_task_range.append(self.task_range[i])

        return node_range, quality, separate_task_range


    def separate_robots(self, robot):
        length = len(self.task_range)
        node_range = []
        quality = []
        computations = []

        for i in range(0, length):
            if (self.robots_range[i] == robot):
                quality.append(self.quality[i])
                node_range.append(self.task_range[i]+self.depots_range[i])
                computations.append(self.computations[i])

        return node_range, quality, computations




    def plot_computation_time_graph_vs_KT(self):
        '''
        Plotting runtime vs robots, tasks
        :return: Plot
        '''

        # # Surface plot TODO: computations should be a matrix
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        # Plot the surface.
        surf = ax.plot_surface(np.array(self.robots_range), np.array(self.task_range), np.array(self.computations), cmap=cm.coolwarm, linewidth=0, antialiased=False)

        # Customize the z axis.
        # ax.zaxis.set_major_locator(LinearLocator(10))
        # ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

        # Managing x,y grid lines
        ax.set_xticks(self.robots_range)
        ax.set_yticks(self.task_range)
        ax.set_zticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))

        # Label axes
        ax.set_xlabel('robots')
        ax.set_ylabel('tasks')
        ax.set_zlabel('computation time')

        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.5, aspect=5)

        plt.show()


    def plot_computation_time(self, name , dir):
        '''
        Plotting runtime vs tasks
        :return: Plot
        '''
        self.readfile(name, dir)

        robots = np.unique(self.robots_range)
        tasks = np.unique(self.task_range)
        nodes = [5, 10, 15, 20]
        print(robots, tasks)

        col = ['b', 'g', 'y', 'r', 'm', 'c', 'k']
        i = 0
        entries = []
        all_data = []
        ax = plt.gca()

        # for r in robots:
        for node in nodes:
            # entries.append('Robots:' + str(int(r)))
            # node_range, quality, computations = self.separate_robots(r)

            entries.append('Nodes:' + str(int(node)))
            node_range, quality, computations, separate_task_range, separate_depot_range = self.separate_nodes(node)

            x = np.array(separate_task_range)
            y = np.array(computations)
            # plt.plot(x, y,  color=col[i])
            all_data.append(y)

            # x = np.array(separate_depot_range)
            # plt.plot(x, y,'--', color=col[i])

            i = i + 1

            # def exponential_func(x, a, b, c):
            #     return a * np.exp(-b * x) + c
            #
            # popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))
            #
            # xx = np.linspace(tasks[0], tasks[-1], 1000)
            # yy = exponential_func(xx, *popt)
            # plt.plot(xx, yy, '--', color=col[i])

        ax.boxplot(all_data, vert=True,  # vertical box alignment
                   patch_artist=True,  # fill with color
                   labels=entries)
        plt.grid(True)
        # plt.legend((entries), loc='upper left')
        pylab.title('Effect on computation time with change in number of nodes')
        # Label axes
        ax.set_xlabel('Number of Nodes')
        ax.set_ylabel('Computation Time (secs)')

        # fig = plt.gcf()
        # py.plot_mpl(fig, filename='TOPTW_ExponentialFit')
        plt.show()

    def plot_compare_quality(self, name, dir, name2, dir2):
        self.readfile(name, dir)

        tasks = np.unique(self.task_range)
        robots = np.unique(self.robots_range)
        print(tasks, robots)
        nodes = [5, 10, 15, 20]#, 25, 30]
        col = ['b', 'g', 'y', 'r', 'm', 'c', 'k']
        i = 0
        entries=[]
        for n in nodes:
            node_range, quality, computations, separate_tasks = self.separate_nodes(n)

            x = np.array(separate_tasks)
            y = np.array(quality)

            plt.plot(x, y,'o', color=col[i])
            plt.yscale('log')
            entries.append('MILP Nodes:'+str(n))
            plt.grid(True)
            # plt.title('Tasks vs Runtime for R=2, N=15')
            # plt.legend(('Without flow constraints', 'Exponential fit - without flow'))
            ax = plt.gca()
            # Label axes
            plt.xticks(tasks)
            ax.set_xlabel('Number of Tasks')
            ax.set_ylabel('Quality')

            self.robots_range = []
            self.task_range = []
            self.depots_range = []
            self.L = []
            self.Tmax_range = []
            self.quality = []

            self.readfileH(name2, dir2)

            node_range, quality, separate_tasks = self.separate_nodesH(n)

            x = np.array([i+0.3 for i in separate_tasks])
            y = np.array(quality)

            plt.plot(x, y, 'x', color=col[i])
            entries.append('Heuristics Nodes:' + str(n))

            i = i + 1

        plt.legend((entries), loc='upper left')
        plt.show()
        fig = plt.gcf()
        plotly_fig = tls.mpl_to_plotly(fig)
        py.iplot(plotly_fig, filename='TOPF_ExponentialFit')


''' Ananlysis class to infer trends in computation time'''
class Analysis_TOPTW:
    def __init__(self):
        '''
        Initialize
        '''
        self.robots_range = []
        self.task_range = []
        self.Tmax_range = []
        self.computations = []
        self.quality = []

    def readFile(self, filename, location):
        # Parse the .csv file
        csvFile = open(location+filename)
        data = list(csv.reader(csvFile))
        csvFile.close()

        for row in data:
            row = list(map(float, row))
            #print(row[0])
            self.robots_range.append(row[0])
            self.task_range.append(row[1])
            self.Tmax_range.append(row[2])
            self.computations.append(row[3])
            self.quality.append(row[4])

    def separate_nodes(self, nodes):
        length = len(self.task_range)
        quality = []
        computations = []
        separate_task_range = []

        for i in range(0, length):
           if (self.task_range[i] == nodes):
               quality.append(self.quality[i])
               computations.append(self.computations[i])
               separate_task_range.append(self.task_range[i])

        return quality, computations, separate_task_range

    def separate_robots(self, robot):
        length = len(self.task_range)
        node_range = []
        quality = []
        computations = []

        for i in range(0, length):
            if (self.robots_range[i] == robot):
                quality.append(self.quality[i])
                node_range.append(self.task_range[i])
                computations.append(self.computations[i])

        return node_range, quality, computations

    def plot_computation_time(self, filename, directory):
        '''
        Plotting runtime vs tasks
        :return: Plot
        '''
        self.readFile(filename, directory)

        robots = np.unique(self.robots_range)
        tasks = np.unique(self.task_range)
        print(robots, tasks)
        nodes = tasks

        col = ['b', 'g', 'y', 'r', 'm', 'c', 'k']
        i=0
        entries = []
        # for r in robots:
        for node in nodes:
            # entries.append('Robots:' + str(int(r)))
            # node_range, quality, computations = self.separate_robots(r)

            entries.append('Nodes:' + str(int(node)))
            quality, computations, separate_task_range = self.separate_nodes(node)

            x = np.array(separate_task_range)
            y = np.array(computations)

            # def exponential_func(x, a, b, c):
            #     return a * np.exp(-b * x) + c
            #
            # popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))
            #
            # xx = np.linspace(tasks[0], tasks[-1], 1000)
            # yy = exponential_func(xx, *popt)

            plt.plot(x, y, 'o', color=col[i])
            i = i+1
            # plt.plot(xx, yy, '--', color=col[i])
            # plt.hold(True)

        # plt.hold(False)
        plt.grid(True)
        # plt.legend((entries),loc='upper left')
        pylab.title('Effect on computation time with change in number of tasks')
        ax = plt.gca()
        # Label axes
        ax.set_xlabel('Number of Tasks')
        ax.set_ylabel('Computation Time (secs)')

        # fig = plt.gcf()
        # py.plot_mpl(fig, filename='TOPTW_ExponentialFit')
        plt.show()



''' Ananlysis class to infer trends in computation time'''
class Analysis_Heuristics:
    def __init__(self, filename, location):
        '''
        Initialize
        :param filename: .csv filename
        :param location: path to the file
        '''

        # noOfRobots, noOfTasks, noOfDepots, L, T_max, cost, seed

        self.filename = filename
        self.directory = location
        self.robots_range = []
        self.task_range = []
        self.depots_range = []
        self.L = []
        self.Tmax_range = []

        self.quality = []

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
            self.quality.append(row[5])

    def plot_T(self):

        tasks = np.unique(self.task_range)
        x = np.array(self.task_range)
        y = np.array(self.quality)

        def exponential_func(x, a, b, c):
            return a * np.exp(-b * x) + c

        # popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))

        # xx = np.linspace(tasks[0], tasks[-1], 1000)
        # yy = exponential_func(xx, *popt)

        plt.plot(x, y, 'o', color='b')
        # plt.plot(xx, yy, color='g')
        plt.grid(True)
        # pylab.title('Exponential Fit')
        ax = plt.gca()
        # Label axes
        ax.set_xlabel('Number of Tasks')
        ax.set_ylabel('Path Cost')
        # ax.set_axis_bgcolor((0.898, 0.898, 0.898))

        fig = plt.gcf()
        py.plot_mpl(fig, filename='Heuristics_ExponentialFit.html')

    def plot_N(self):
        length = len(self.task_range)
        nodes = [5, 10, 15]
        node_range = []
        quality =[]

        avg_time = []

        for i in range(0,length):
            if(self.task_range[i] + self.depots_range[i] == nodes[0]):
                quality.append(self.quality[i])
                node_range.append(nodes[0])

            if (self.task_range[i] + self.depots_range[i] == nodes[1]):
                quality.append(self.quality[i])
                node_range.append(nodes[1])

            if (self.task_range[i] + self.depots_range[i] == nodes[2]):
                quality.append(self.quality[i])
                node_range.append(nodes[2])


        x = np.array(node_range)
        y = np.array(quality)


        # popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))

        # xx = np.linspace(tasks[0], tasks[-1], 1000)
        # yy = exponential_func(xx, *popt)

        plt.plot(x, y, 'o', color='b')
        # plt.plot(xx, yy, color='g')
        plt.grid(True)
        # pylab.title('Exponential Fit')
        ax = plt.gca()
        # Label axes
        ax.set_xlabel('Number of Tasks')
        ax.set_ylabel('Path Cost')
        # ax.set_axis_bgcolor((0.898, 0.898, 0.898))

        fig = plt.gcf()
        py.plot_mpl(fig, filename='Heuristics_ExponentialFit.html')



def main():
    print("Analysis of experiments performed")

    # .csv file and its location
    name = 'topf_experiment_milp.csv'
    dir = '../../../../experiments/all_4_14/csv/'
    name1 = 'topf_experiment_heuristic.csv'
    dir1 = '../../../../experiments/all_4_14/csv/'

    '''TOPF Analysis'''
    # object
    # inst = Analysis_TOPF()
    # # Plot TOPF experiment data
    # # inst.plot_compare_quality(name, dir, name1, dir1)
    # inst.plot_computation_time(name, dir)

    '''TOPTW Analysis'''
    name = 'toptw_experiment.csv'
    inst = Analysis_TOPTW()
    inst.plot_computation_time(name, dir)


    '''Heuristics Analysis'''
    # inst = Analysis_Heuristics(name,dir)
    # inst.plot_N()

if __name__ == "__main__":
        main()
