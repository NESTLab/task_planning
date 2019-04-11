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

    def separate_nodes(self, nodes):
        length = len(self.task_range)
        node_range = []
        quality = []
        computations = []
        separate_task_range = []

        for node in nodes:
            for i in range(0, length):
                if (self.task_range[i] + self.depots_range[i] == node):
                    quality.append(self.quality[i])
                    node_range.append(node)
                    computations.append(self.computations[i])
                    separate_task_range.append(self.task_range[i])

        return node_range, quality, computations, separate_task_range


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


    def plot_computation_time_graph_vs_T(self, name , dir):
        '''
        Plotting runtime vs tasks
        :return: Plot
        '''
        self.readfile(name, dir)
        # print(np.array(self.task_range))
        # # print(self.computations)
        # # Graph
        # fig = plt.figure()
        # ax = fig.gca()
        # # Plot the graph
        # plt.scatter(np.array(self.task_range), np.array(self.computations))
        #
        # # Managing x,y grid lines
        # ax.set_xticks(self.task_range)
        # # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))
        #
        # # Label axes
        # ax.set_xlabel('tasks')
        # ax.set_ylabel('computation time')
        #
        #
        # plt.show()

        tasks = np.unique(self.task_range)
        print(tasks)

        x = np.array(self.task_range)
        y = np.array(self.computations)

        def exponential_func(x, a, b, c):
            return a * np.exp(-b * x) + c

        popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))

        xx = np.linspace(tasks[0], tasks[-1], 1000)
        yy = exponential_func(xx, *popt)

        plt.plot(x, y, 'o', color='b')
        plt.plot(xx, yy, color='g')
        plt.grid(True)
        # pylab.title('Exponential Fit')
        ax = plt.gca()
        # Label axes
        ax.set_xlabel('Number of Tasks')
        ax.set_ylabel('Computation Time (secs)')
        # ax.set_axis_bgcolor((0.898, 0.898, 0.898))

        fig = plt.gcf()
        py.plot_mpl(fig, filename='TOPF_ExponentialFit')


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


    def plot_compare_flow(self, name, dir, name2, dir2):
        self.readfile(name, dir)

        tasks = np.unique(self.task_range)
        robots = np.unique(self.robots_range)
        print(tasks)
        # nodes = [5, 10, 15, 20, 25, 30]
        # node_range, quality, computations, separate_tasks = self.separate_nodes(nodes)

        colors = ['r', 'g', 'y','k']
        i = 0
        for r in robots:
            if i>0:
                self.robots_range = []
                self.task_range = []
                self.depots_range = []
                self.L = []
                self.Tmax_range = []
                self.computations = []
                self.quality = []
                self.readfile(name, dir)

            new_nodes, quality, computations = self.separate_robots(r)
            print(new_nodes)
            x = np.array(new_nodes)
            y = np.array(quality)

            def exponential_func(x, a, b, c):
                return a * np.exp(-b * x) + c

            # popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))
            #
            # xx = np.linspace(tasks[0], tasks[-1], 1000)
            # yy = exponential_func(xx, *popt)

            plt.plot(x, y, 'o', color='b')
            # plt.plot(xx, yy, color='g')
            plt.grid(True)
            # plt.title('Tasks vs Runtime for R=2, N=15')
            # plt.legend(('Without flow constraints', 'Exponential fit - without flow'))
            ax = plt.gca()
            # Label axes
            ax.set_xlabel('Number of Tasks')
            # ax.set_ylabel('Computation Time (secs)')
            ax.set_ylabel('Quality')


            self.robots_range = []
            self.task_range = []
            self.depots_range = []
            self.L = []
            self.Tmax_range = []
            self.computations = []
            self.quality = []

            self.readfile(name2, dir2)

            tasks = np.unique(self.task_range)
            # node_range, quality, computations, separate_tasks = self.separate_nodes(nodes)
            new_nodes, quality, computations = self.separate_robots(r)

            x = np.array(new_nodes)
            y = np.array(quality)

            #
            # popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))
            #
            # xx = np.linspace(tasks[0], tasks[-1], 1000)
            # yy = exponential_func(xx, *popt)

            plt.plot(x, y, 'x', color='b')
            # plt.plot(xx, yy, color='y')
            break
        # plt.legend(('Without flow constraints', 'Exponential fit - without flow','With flow constraints', 'Exponential fit - with flow'),loc='upper left')
        # plt.legend(('MILP', 'Exponential fit - MILP','Heuristics', 'Exponential fit - Heuristics'),loc='upper left')
        plt.show()
        fig = plt.gcf()
        plotly_fig = tls.mpl_to_plotly(fig)
        py.iplot(plotly_fig, filename='TOPF_ExponentialFit')


''' Ananlysis class to infer trends in computation time'''
class Analysis_TOPTW:
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
        self.Tmax_range = []
        self.computations = []
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
            self.Tmax_range.append(row[2])
            self.computations.append(row[3])

            self.quality.append(row[4])



    def plot_computation_time_graph_vs_T_boxplot(self):
        '''
        Plotting runtime vs tasks
        :return: Plot
        '''

        K = [2, 3, 4, 5, 8, 10]
        task_temp2 = []
        computation2 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[0]):
                task_temp2.append(self.task_range[i])
                computation2.append(self.computations[i])
        print(task_temp2)
        print(computation2)

        task_temp3 = []
        computation3 = []
        for i in range(0,len(self.robots_range)):
            if(self.robots_range[i] == K[1]):
                task_temp3.append(self.task_range[i])
                computation3.append(self.computations[i])
        print(task_temp3)
        print(computation3)

        task_temp4 = []
        computation4 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[2]):
                task_temp4.append(self.task_range[i])
                computation4.append(self.computations[i])

        task_temp5 = []
        computation5 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[3]):
                task_temp5.append(self.task_range[i])
                computation5.append(self.computations[i])

        task_temp8 = []
        computation8 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[4]):
                task_temp8.append(self.task_range[i])
                computation8.append(self.computations[i])

        task_temp10 = []
        computation10 = []
        for i in range(0, len(self.robots_range)):
            if (self.robots_range[i] == K[5]):
                task_temp10.append(self.task_range[i])
                computation10.append(self.computations[i])

        # Graph
        fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(12, 6))

        # Plot the graph
        axes[0, 0].scatter(np.array(task_temp2), np.array(computation2))
        axes[0, 1].scatter(np.array(task_temp3), np.array(computation3))
        axes[0, 2].scatter(np.array(task_temp4), np.array(computation4))
        axes[1, 0].scatter(np.array(task_temp5), np.array(computation5))
        axes[1, 1].scatter(np.array(task_temp8), np.array(computation8))
        axes[1, 2].scatter(np.array(task_temp10), np.array(computation10))

        # Managing x,y grid lines
        axes[0, 0].set_title('robots = 2')
        axes[0, 1].set_title('robots = 3')
        axes[1, 2].set_title('robots = 4')
        axes[1, 0].set_title('robots = 5')
        axes[1, 1].set_title('robots = 8')
        axes[1, 2].set_title('robots = 10')
        axes[0, 0].set_xticks(task_temp2)
        axes[0, 1].set_xticks(task_temp3)
        axes[0, 2].set_xticks(task_temp4)
        axes[1, 0].set_xticks(task_temp5)
        axes[1, 1].set_xticks(task_temp8)
        axes[1, 2].set_xticks(task_temp10)


        # # Label axes
        axes[0, 0].set_xlabel('tasks')
        axes[0, 0].set_ylabel('computation time')

        fig.tight_layout()
        plt.show()

    def plot_computation_time_graph_vs_T(self):
        '''
        Plotting runtime vs tasks
        :return: Plot
        '''

        # print(np.array(self.task_range))
        # print(self.computations)

        tasks = np.unique(self.task_range)
        print(tasks)
        avg_time = []

        for i in tasks:
            ind = [x for x in range(len(self.task_range)) if self.task_range[x] == int(i)]
            print(ind)
            temp_computations = [self.computations[y] for y in ind]
            avg_time.append(np.mean(temp_computations))
        print(avg_time)


        # # Graph
        # fig = plt.figure()
        # ax = fig.gca()
        # plt.grid(True)
        # # Plot the graph
        # plt.scatter(np.array(self.task_range), np.array(self.computations))
        # plt.plot(tasks, avg_time)
        #
        # # Managing x,y grid lines
        # ax.set_xticks(self.task_range)
        # # ax.set_yticks(np.linspace(np.min(self.computations), np.max(self.computations), len(self.task_range)))
        #
        # # Label axes
        # ax.set_xlabel('Number of Tasks')
        # ax.set_ylabel('Computation Time (secs)')
        # plt.show()


        # x = np.array([399.75, 989.25, 1578.75, 2168.25, 2757.75, 3347.25, 3936.75, 4526.25, 5115.75, 5705.25])
        # y = np.array([109, 62, 39, 13, 10, 4, 2, 0, 1, 2])

        x = np.array(self.task_range)
        y = np.array(self.computations)
        def exponential_func(x, a, b, c):
            return a * np.exp(-b * x) + c

        popt, pcov = curve_fit(exponential_func, x, y, p0=(1, 1e-6, 1))

        xx = np.linspace(tasks[0], tasks[-1], 1000)
        yy = exponential_func(xx, *popt)

        plt.plot(x, y, 'o', color='b')
        plt.plot(xx, yy, color='g')
        plt.grid(True)
        # pylab.title('Exponential Fit')
        ax = plt.gca()
        # Label axes
        ax.set_xlabel('Number of Tasks')
        ax.set_ylabel('Computation Time (secs)')
        # ax.set_axis_bgcolor((0.898, 0.898, 0.898))

        fig = plt.gcf()
        py.plot_mpl(fig, filename='TOPTW_ExponentialFit')




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



def main():
    print("Analysis of experiments performed")

    # .csv file and its location
    name = 'compare_topf_experiment_milp.csv'
    dir = '../../../../experiments/compare_topf_fail/'
    name1 = 'compare_topf_experiment_heuristic.csv'
    dir1 = '../../../../experiments/compare_topf_fail/'

    '''TOPF Analysis'''
    # object
    inst = Analysis_TOPF()
    # Plot TOPF experiment data
    inst.plot_compare_flow(name, dir, name1, dir1)
    # inst.plot_computation_time_graph_vs_T(name1, dir1)


    '''TOPTW Analysis'''
    # inst = Analysis_TOPTW(name, dir)
    # # inst.plot_computation_time_graph_vs_T_boxplot()
    # inst.plot_computation_time_graph_vs_T()


    '''Heuristics Analysis'''
    # inst = Analysis_Heuristics(name,dir)
    # inst.plot_N()

if __name__ == "__main__":
        main()
