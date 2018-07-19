import math
import csv
import bisect


class Node:
    """The node class contains info on the nodes, such as their id, coordinates and costs. Contains functions for
    comparision to other nodes based on cost, updating the node's costs and parent, and exploring all edges connected
    to the node"""
    def __init__(self, node_id, x, y, optimistic_ctg):
        """creates the node object, different values are used for the special case where the node is the first node in
        the path"""
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

    def __repr__(self):
        """Returns a string that represents the node. This is used to simplify writing the node to path.csv"""
        return str(self.node_id)

    def __lt__(self, other):
        """Tests whether this node is 'less than' another node
        This function is required to use the bisect function to insert the node object into the correct position open
        node list"""
        return self.est_tot_cost < other.est_tot_cost

    def update(self, parent, cost):
        """If the new cost of traveling to this node is lower than the previous cost update the cost and parent of the
        node"""
        global open_nodes
        if cost < self.past_cost:
            if self not in closed_nodes:
                self.past_cost = cost
                self.est_tot_cost = self.optimistic_ctg + self.past_cost
                self.parent_node = parent
                # if the node is already in open nodes remove it so it can be inserted without duplication
                if self in open_nodes:
                    open_nodes.remove(self)
                # insert the node in the correct location, by estimated total cost, in open nodes
                bisect.insort(open_nodes, self)

    def explore(self, edges):
        """Looks through the list of edges for edges connected to the node.
        Runs the update function on the connected node"""
        for edge in edges:
            if edge.id1 == self.node_id:
                nodes[edge.id2-1].update(self.node_id, edge.cost)
            elif edge.id2 == self.node_id:
                nodes[edge.id1-1].update(self.node_id, edge.cost)
        # once all edges connected to the node have been explored we close it
        closed_nodes.append(self)
        open_nodes.remove(self)


class Edge:
    """Simple data class for storing edge data for convenience"""
    def __init__(self, id1, id2, cost):
        self.id1 = int(id1)
        self.id2 = int(id2)
        self.cost = float(cost)


# create empty lists of nodes and edges
nodes = []
open_nodes = []
closed_nodes = []
edge_list = []

# import nodes.csv and edges.csv
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

# explore each node in open nodes
while open_nodes:
    open_nodes[0].explore(edge_list)

# create the path backwards starting with the end node
path = [nodes[-1]]
while True:
    if path[0].parent_node:
        # if the first node in the path has a parent node the parent node is added to the start of the path
        path = [nodes[path[0].parent_node - 1]] + path
    else:
        # if the first node in the path has no parent node it is the start node and we exit the loop
        break

# write path to path.csv
with open('path.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(path)






