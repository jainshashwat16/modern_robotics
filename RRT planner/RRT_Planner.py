import random
import csv
from anytree import NodeMixin


"""
This code takes a .csv file containing the location and diameter of round obstacles and uses the Rapidly Exploring
Random Tree algorithm to produce a path from (-.5,-.5) to (.5,.5). The code uses the uniform distribution over 
[-0.5, 0.5] x [-0.5, 0.5] for its random sampling. Every 10 samples the code selects the goal as the sample. 
For local planning the code uses a straight line between nodes.

Example input:
A .csv file titled obstacles.csv located in the directory from which the code is executed containing the following
columns are x, y, diameter:

0.0, 0.0, 0.2
0.0, 0.1, 0.2
0.3, 0.2, 0.2
-0.3, -0.2, 0.2
-0.1, -0.4, 0.2
-0.2, 0.3, 0.2
0.3, -0.3, 0.2
0.1, 0.4, 0.2

Example output:
3 .csv files located in the directory from which the code is executed

nodes.csv containing:
#ID,X,Y
1,-0.5,-0.5
2,0.48388486950392096,0.16353641408734887
3,-0.45385608924722,0.2397028599163319
4,0.45004976981087363,-0.10680190856020566
5,-0.4746829351255277,-0.010926822324482721
6,-0.39473907252811113,0.45540635796838624
7,-0.1736030378307707,0.06432260083542196
8,-0.42727549833580236,-0.43808443391652996
9,0.3831067087526274,-0.11948453630635691
10,0.003842617135883075,-0.2653641969478796
11,0.5,0.5

path.csv containing:
1,2,11

edges.csv containing:
#ID1,ID2,COST
2,1,1.1867223808703724
3,1,0.7411407298671158
4,2,0.2724474677128267
5,3,0.25149352900098737
6,3,0.2236578206494466
7,5,0.3103410708251807
8,1,0.09551120596946953
9,4,0.06813385700513018
10,7,0.37440665739160184
11,2,0.33684928688674054
"""


class Node(NodeMixin):
    """Data class that stores node coordinates and parents. Subclasses the anytree NodeMixin class to represent the
    tree"""
    count = 0

    def __init__(self, x, y, parent=None):
        super(Node, self).__init__()
        self.x = x
        self.y = y
        self.parent = parent
        Node.count += 1
        self.node_id = Node.count

    def __repr__(self):
        """Returns a representation of the node, in this case its coordinates"""
        return f"({self.x},{self.y})"


class Edge:
    """Data class for representing edges"""
    def __init__(self, node_1, node_2):
        self.node_1 = node_1
        self.node_2 = node_2
        self.cost = node_distance(node_1, node_2)


def nearest(samp, nodes):
    """Returns the nearest node to the sample."""
    # initialize the lowest distance to the maximum distance between points in the space [-.5,.5]x [-.5,.5]y
    lowest = 2 ** .5
    for node in nodes:
        distance = node_distance(samp, node)
        if distance < lowest:
            lowest = distance
            nearest_node = node
    return nearest_node


def node_distance(node_1, node_2):
    """Returns the straight line distance between two nodes."""
    return ((node_1.x - node_2.x) ** 2 + (node_1.y - node_2.y) ** 2) ** .5


def collision_free(node_1, node_2, obstacles):
    """Returns true if no collision is detected. This uses the intersection finding approach described here:
    http://mathworld.wolfram.com/Circle-LineIntersection.html"""
    for obstacle in obstacles:
        n1_x = node_1.x - obstacle[0]
        n1_y = node_1.y - obstacle[1]
        n2_x = node_2.x - obstacle[0]
        n2_y = node_2.y - obstacle[1]

        d_x = n2_x - n1_x
        d_y = n2_y - n1_y
        d_r = (d_x ** 2 + d_y ** 2) ** .5
        D = n1_x * n2_y - n2_x * n1_y
        discriminant = (obstacle[2] ** 2) * (d_r ** 2) - (D ** 2)

        # check if intersection occurs on an infinite line:
        if discriminant >= 0:
            # calculate the locations of the intersections. If the line is tangent to the circle x_1 = x_2 and y_1 = y_2
            x_1 = (D * d_y + sgn(d_y) * d_x * discriminant ** .5) / (d_r ** 2)
            y_1 = (-D * d_x + abs(d_y) * discriminant ** .5) / (d_r ** 2)
            x_2 = (D * d_y - sgn(d_y) * d_x * discriminant ** .5) / (d_r ** 2)
            y_2 = (-D * d_x - abs(d_y) * discriminant ** .5) / (d_r ** 2)

            # check if if the intersection points fall between the end points of the line segment
            # if the line is not vertical (node_1.x is not equal to node_2.x we check if either x value is between the
            # end points, if it is vertical we check y instead
            if n1_x != n2_x:
                # check if at least one of the intersections is between the endpoints
                if bound(x_1, x_2, n1_x, n2_x):
                    return False
            elif bound(y_1, y_2, n1_y, n2_y):
                return False
    return True


def bound(v_1, v_2, v_3, v_4):
    """Checks if one or both of the two values are between the other two"""
    # put line endpoints in order
    endpoints = sorted([v_3, v_4])
    if (endpoints[0] <= v_1 <= endpoints[1]) or (endpoints[0] <= v_2 <= endpoints[1]):
        return True
    else:
        return False


def sgn(value):
    """Used in the collision detection calculations. See the MathWorld link in the collision_free docstring"""
    if value < 0:
        return -1
    else:
        return 1


# import obstacles.csv
obstacles = []
edges = []
with open('obstacles.csv') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        # if the row is not a comment create a new node object and add it to the list of nodes
        if not row[0].startswith('#'):
            # add the obstacle to the list of obstacles, and change the diameter to radius
            obstacles.append([float(row[0]), float(row[1]), float(row[2])/2.0])

tree_max = 200

# 1: initialize search tree T with x_start
node_list = [Node(-.5, -.5)]
# 2: while T is less than the maximum tree size:
x_samp = None
while len(node_list) < tree_max:
    # 3: x_samp <- sample from X
    if len(node_list) % 10 == 0 and (x_samp.x != .5 and x_samp.y != .5):
        # every 10 nodes we sample the goal, checks that the previous sample wasn't also the goal to prevent infinite
        # goal sampling loops when a collision occurs
        x_samp = Node(.5, .5)
    else:
        x_samp = Node(random.uniform(-.5, .5), random.uniform(-.5, .5))
    # 4: x_nearest <- nearest node in T to x_samp
    x_nearest = nearest(x_samp, node_list)
    # 5: employ a local planner to find a motion from x_nearest to x_new in the direction of x_samp
    motion = Edge(x_samp, x_nearest)
    # 6: if the motion is collision-free:
    if collision_free(x_samp, x_nearest, obstacles):
        edges.append(motion)
        # 7: add x_new to T with an edge from x_nearest to x_new
        x_samp.parent = x_nearest
        node_list.append(x_samp)
        # 8: if x_new is in X_goal then:
        if x_samp.x == .5 and x_samp.y == .5:
            # return SUCCESS and the motion to x_new
            print("SUCCESS")
            print(x_samp.ancestors)
            break
    else:
        Node.count -= 1

# write CSV output
with open('nodes.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['#ID', 'X', 'Y'])
    for node in node_list:
        writer.writerow([node.node_id, node.x, node.y])

with open('edges.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['#ID1', 'ID2', 'COST'])
    for edge in edges:
        writer.writerow([edge.node_1.node_id, edge.node_2.node_id, edge.cost])

with open('path.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    path = []
    for node in x_samp.ancestors:
        path.append(node.node_id)
    path.append(x_samp.node_id)
    writer.writerow(path)

