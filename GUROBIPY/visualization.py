'''
Creates interactive graphs using plotly for optimized routes
'''

# Importing existing python modules
import random as rnd
import csv
import pprint
import numpy as np
import networkx as nx
import plotly.offline as py
import plotly.graph_objs as go
import matplotlib as mplt
import matplotlib.animation as animation


'''Visualization for TOPF'''


class Visualization_TOPF:
    def __init__(self, K, T, D, S, T_loc, D_loc, c):
        self.K = K
        self.T = T
        self.D = D
        self.S = S
        self.N = self.T + self.D
        self.T_loc = T_loc
        self.D_loc = D_loc
        self.c = c
        self.arcsInOrder = {k: [] for k in self.K}

    def preprocessing(self, model):
        v = model.getVars()

        finalArcs = {k: [] for k in self.K}
        finalArcsWeighted = {k: [] for k in self.K}
        remainingFuel = {t: 0 for t in self.T}
        for i in range(len(v)):
            if v[i].x >= 0.9 and v[i].varName[0] == 'x':
                x, y, k = v[i].VarName.split(',')
                x = x.replace("x[", "")
                k = k.replace("]", "")
                finalArcs[k].append((x, y))
                finalArcsWeighted[k].append((x, y, self.c[x, y]))
            if v[i].x >= 0.9 and v[i].varName[0] == 'r':
                x, y = v[i].VarName.split('[')
                y = y.replace("]", "")
                remainingFuel[y] = v[i].x

        # Create a graph for each robot
        G = {k: nx.DiGraph() for k in self.K}
        # Add all nodes in the graph for each robot
        for k in self.K:
            G[k].add_nodes_from(self.N)
            G[k].add_weighted_edges_from(finalArcsWeighted[k])
            # print("Nodes in G["+k+"]: ", G[k].nodes(data=True))
            # print("Edges in G["+k+"]: ", G[k].edges(data=True))

        # Now compute the paths in the above graphs
        for k in self.K:
            self.arcsInOrder[k] = list(nx.edge_dfs(G[k], source=self.S[0]))
        pprint.pprint(self.arcsInOrder)

        # Also compute the length of the path
        l = {k: 0 for k in self.K}
        for k in self.K:
            for arc in self.arcsInOrder[k]:
                l[k] += self.c[arc]
            print("Length [%s] = %.2f" % (k, l[k]))

        return remainingFuel

    def taskNodesTrace(self, remainingFuel):
        taskTrace = go.Scatter(
            text=[],
            hovertext=[],
            x=[],
            y=[],
            mode='markers+text',
            textposition='bottom right',
            # hoverinfo='text',
            name='<br>Task Locations<br>',
            marker=dict(
                size=6,
                color='blue',
                line=dict(
                    # color='rgba(217, 217, 217, 0.14)',
                    width=0.5
                ),
                opacity=0.8
            )
        )

        for t in self.T_loc:
            x, y = self.T_loc.get(t)
            disp_text = 'NodeID: ' + t + '<br>f_left: ' + \
                "{0:.2f}".format(remainingFuel[t])
            taskTrace['x'] += tuple([x])
            taskTrace['y'] += tuple([y])
            taskTrace['text'] += tuple([t])
            taskTrace['hovertext'] += tuple([disp_text])
        return taskTrace

    def startNodesTrace(self, S_loc):
        startTrace = go.Scatter(
            text=[],
            hovertext=[],
            x=[],
            y=[],
            mode='markers+text',
            textposition='top center',
            # hoverinfo='text',
            name='Refueling Locations<br>',
            marker=dict(
                size=12,
                color='green',
                line=dict(
                    # color='rgba(217, 217, 217, 0.14)',
                    width=0.5
                ),
                opacity=0.8
            )
        )

        for s in S_loc:
            x, y = S_loc.get(s)
            # + '<br>f_left: ' + "{0:.2f}".format(f_left)
            disp_text = 'NodeID: ' + s
            startTrace['x'] += tuple([x])
            startTrace['y'] += tuple([y])
            startTrace['text'] += tuple([s])
            startTrace['hovertext'] += tuple([disp_text])
        return startTrace

    def edgeTrace(self, S_loc, arcsInOrder):
        colors = ['rgb(31, 119, 180)', 'rgb(255, 127, 14)',
                  'rgb(44, 160, 44)', 'rgb(214, 39, 40)',
                  'rgb(148, 103, 189)', 'rgb(140, 86, 75)',
                  'rgb(227, 119, 194)', 'rgb(127, 127, 127)',
                  'rgb(188, 189, 34)', 'rgb(23, 190, 207)']
        edge_trace = go.Scatter(
            x=[],
            y=[],
            text=[],
            line=dict(width=1, color=colors[rnd.randint(
                0, len(colors) - 1)], dash='dash'),
            hoverinfo='none',
            showlegend=True,
            mode='lines')

        node_info_trace = go.Scatter(
            text=[],
            x=[],
            y=[],
            mode='markers',
            hoverinfo='text',
            # name = 'Edge Info',
            showlegend=False,
            marker=dict(
                size=12,
                symbol='pentagon-open-dot',
                color='mistyrose',
                line=dict(
                    # color='rgba(217, 217, 217, 0.14)',
                    width=0.5
                ),
                opacity=0.8
            )
        )

        N_loc = {**S_loc, **self.T_loc}
        for arc in arcsInOrder:
            x0, y0 = N_loc.get(arc[0])
            x1, y1 = N_loc.get(arc[1])
            edge_trace['x'] += tuple([x0, x1])
            edge_trace['y'] += tuple([y0, y1])

            node_info_trace['x'] += tuple([(x0 + x1) / 2])
            node_info_trace['y'] += tuple([(y0 + y1) / 2])
            node_info_trace['text'] += tuple(
                ["Weight: " + "{0:.2f}".format(np.linalg.norm(np.array([x0, y0]) - np.array([x1, y1])))])
        return edge_trace, node_info_trace

    def drawArena(self, remainingFuel, isEdge=1):
        task_trace = self.taskNodesTrace(remainingFuel)
        start_trace = self.startNodesTrace(self.D_loc)
        # end_trace = endNodesTrace(E_loc)

        data = [task_trace, start_trace]

        if isEdge:
            for k in self.arcsInOrder:
                edge_trace, node_info_trace = self.edgeTrace(
                    self.D_loc, self.arcsInOrder[k])
                edge_trace.name = str(k)
                data.append(edge_trace)
                data.append(node_info_trace)

        # if isEdge:
        #    print(edge_trace['x'][0])

        layout = go.Layout(
            title='Arena',
            hovermode='closest',
            xaxis=dict(
                title='X-Coord',
                range=[0, 100]
                # ticklen= 5,
                # zeroline= False,
                # gridwidth= 2,
            ),
            yaxis=dict(
                title='Y-Coord'
                # ticklen= 5,
                # gridwidth= 2,
            ),
            showlegend=True

        )
        fig = go.Figure(data=data, layout=layout)
        return fig

    def save_plot_topf_milp(self, model, name, auto_open_flag=0):
        remainingFuel = self.preprocessing(model)

        fig = self.drawArena(remainingFuel, 1)
        py.plot(fig, filename=name + '.html',
                auto_open=auto_open_flag, include_plotlyjs='cdn')

        with open(name + '.csv', 'w') as csv_file:
            writer = csv.writer(csv_file)
            for key, value in self.arcsInOrder.items():
                writer.writerow([key, value])

        return self.arcsInOrder

    def save_plot_topf_heuristic(self, arcsInOrder, name, auto_open_flag):
        self.arcsInOrder = arcsInOrder
        remainingFuel = {t: 0 for t in self.T}
        fig = self.drawArena(remainingFuel, 1)
        py.plot(fig, filename=name + '.html',
                auto_open=auto_open_flag, include_plotlyjs='cdn')

        with open(name+'.csv', 'w') as csv_file:
            writer = csv.writer(csv_file)
            for key, value in self.arcsInOrder.items():
                writer.writerow([key, value])


'''Visualization for TOPTW'''
# TODO: incomplete


class Visualization_TOPTW:
    def __init__(self, model):

        self.v = model.getVars()
