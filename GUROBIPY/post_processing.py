# Importing existing python modules
import csv
import numpy as np
import os



class Post_Processing_TOPF:
    def __init__(self):
        data=[]


class Post_Processing_TOPTW:
    def __init__(self, routes, A, D, Q, R, T_loc, S_loc, E_loc, arrival, c, W):
        self.routes = routes
        self.A = A
        self.D = D
        self.Q = Q
        self.R = R
        self.T_loc = T_loc
        self.S_loc = S_loc
        self.E_loc = E_loc
        self.arrival = arrival
        self.c = c
        self.W = W

    def save_trends(self):
        distances =[]
        time_left = []
        rewards_selected = []

        # print(len(self.routes))
        for w in self.W:
            for edge in self.routes[w]:
                if edge[1] != 'E':
                    # print(edge[1])
                    distances.append(self.c.get(edge))
                    task_window = self.A[edge[1]]
                    # print(self.arrival[w, edge[1]].x)
                    time_left.append(task_window[1] - self.arrival[w, edge[1]].x)
                    rewards_selected.append(self.R[edge[1]])
        distances = list(distances)
        # print(distances)


        with open('toptw_analysis_distance.csv', 'a') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerows([[d] for d in distances])

        csvFile.close()

        with open('toptw_analysis_reward.csv', 'a') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerows([[r] for r in rewards_selected])

        csvFile.close()

        with open('toptw_analysis_time.csv', 'a') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerows([[t] for t in time_left])

        csvFile.close()







def main():

    print("Post Processing")



if __name__ == "__main__":
        main()

