import random
import csv
from anytree import NodeMixin


class Node(NodeMixin):
    """Data class that stores node coordinates and parents. Subclasses the any tree NodeMixin class to represent the
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
        return f"({self.x},{self.y})"


class Edge:
    def __init__(self, node_1, node_2):
        self.node_1 = node_1
        self.node_2 = node_2
        self.cost = node_distance(node_1, node_2)


def nearest(samp, nodes):
    """Returns the nearest node to the sample"""
    # initialize the lowest distance to the maximum distance between points in the space [-.5,.5]x [-.5,.5]y
    lowest = 2 ** .5
    for node in nodes:
        distance = node_distance(samp, node)
        if distance < lowest:
            lowest = distance
            nearest_node = node
    return nearest_node


def node_distance(node_1, node_2):
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
    print(f'{v_1}, {v_2}, {endpoints[0]}, {endpoints[1]}')
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
while len(node_list) < tree_max:
    # 3: x_samp <- sample from X
    if len(node_list) % 10 == 0:
        # every 10 nodes we sample the goal
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
        print(f"({x_samp.x},{x_samp.y})")
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
        print("collision detected")

# write CSV output
with open('nodes.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['#ID', 'X', 'Y'])
    for node in node_list:
        writer.writerow([node.node_id, node.x, node.y])

with open('edges.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(['#ID1', 'ID2', 'COST'])
    for edge in edges:
        writer.writerow([edge.node_1.node_id, edge.node_2.node_id, edge.cost])

with open('path.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    path = []
    for node in x_samp.ancestors:
        path.append(node.node_id)
    path.append(x_samp.node_id)
    writer.writerow(path)

