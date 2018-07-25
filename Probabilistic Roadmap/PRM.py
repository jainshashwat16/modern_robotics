import random
import csv
from path_utils import Node, Edge

obstacles = []
edges = []
with open('obstacles.csv') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        # if the row is not a comment create a new node object and add it to the list of nodes
        if not row[0].startswith('#'):
            # add the obstacle to the list of obstacles, and change the diameter to radius
            obstacles.append([float(row[0]), float(row[1]), float(row[2])/2.0])

initial_samples = 20
no_neighbors = 4
edges = []
goal = Node(.5, .5, goal=True)
start = Node(-.5, -.5, start=True)

# 1: for i = 1 to N
for i in range(0, initial_samples):
    # 2: q_i <- sample from Cfree
    new_node = Node(random.uniform(-.5, .5), random.uniform(-.5, .5))
    # 3: add q_i to R
# for i = 1 to N
for node in Node.list:
    # N(q_i) <- k closest neighbors of q_i
    # for each q within N(q_i)
    nearest = node.nearest_neighbors(no_neighbors)
    for neighbor in nearest:
        print(f"{node}, {neighbor}")
        local_path = Edge(node, neighbor)
        if local_path.collision_free:
            edges.append(local_path)
        # if there is a collision-free local path from q to qi and there is not already an edge between them then
            # add an edge from q to q_i to the roadmap R

# write CSV output
with open('nodes.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['#ID', 'X', 'Y', 'COST TO GO'])
    for node in Node.list:
        writer.writerow([node.node_id, node.x, node.y, node.optimistic_ctg])

with open('edges.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['#ID1', 'ID2', 'COST'])
    for edge in edges:
        writer.writerow([edge.node_1.node_id, edge.node_2.node_id, edge.cost])
