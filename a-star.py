import math
import csv
import bisect


class Node:
    def __init__(self, node_id, x, y, optimistic_ctg):
        self.node_id = int(node_id)
        self.x = float(x)
        self.y = float(y)
        self.optimistic_ctg = float(optimistic_ctg)
        if self.node_id == 1:
            self.past_cost = 0
            self.est_tot_cost = self.optimistic_ctg
            open_nodes.append(self)
        else:
            self.past_cost = math.inf
            self.est_tot_cost = math.inf
        self.parent_node = None

    # human readable representation of the node, not needed for the algorithm, but useful for testing
    def __str__(self):
        return f"<{self.node_id}, {self.x}, {self.y}, {self.optimistic_ctg}, {self.past_cost}, {self.est_tot_cost}, {self.parent_node}>"

    def __repr__(self):
        return self.__str__()

    def __lt__(self, other):
        return self.est_tot_cost < other.est_tot_cost

    def update(self, parent, cost):
        global open_nodes
        if self not in closed_nodes:
            self.past_cost = cost
            self.est_tot_cost = self.optimistic_ctg + self.past_cost
            self.parent_node = parent
            if self not in open_nodes:
                bisect.insort(open_nodes, self)

    def explore(self, edges):
        for edge in edges:
            if edge.id1 == self.node_id:
                nodes[edge.id2-1].update(self.node_id, edge.cost)
            elif edge.id2 == self.node_id:
                nodes[edge.id1-1].update(self.node_id, edge.cost)
        closed_nodes.append(self)
        open_nodes.remove(self)


class Edge:
    def __init__(self, id1, id2, cost):
        self.id1 = int(id1)
        self.id2 = int(id2)
        self.cost = float(cost)


# create empty list of nodes
nodes = []
open_nodes = []
closed_nodes = []
edge_list = []

# import nodes.csv
with open('nodes.csv') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        # if the row is not a comment create a new node object and add it to the list of nodes
        if not row[0].startswith('#'):
            nodes.append(Node(row[0], row[1], row[2], row[3]))

with open('edges.csv') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        # if the row is not a comment create an edge object and add it to the list of edges
        if not row[0].startswith('#'):
            edge_list.append(Edge(row[0], row[1], row[2]))

while open_nodes:
    open_nodes[0].explore(edge_list)


path = [nodes[-1]]
while True:
    if not path[0].parent_node:
        break
    else:
        path = [nodes[path[0].parent_node-1]] + path

for node in path:
    print(node.node_id)