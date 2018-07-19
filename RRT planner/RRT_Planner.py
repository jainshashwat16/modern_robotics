import random
import csv
from anytree import NodeMixin


class Node(NodeMixin):
    """Data class that stores node coordinates and parents. Subclasses the any tree NodeMixin class to represent the
    tree"""
    def __init__(self, x, y, parent=None):
        super(Node, self).__init__()
        self.x = x
        self.y = y
        self.parent = parent

    def __repr__(self):
        return f"({self.x},{self.y})"


def nearest(samp, nodes):
    """Returns the nearest node to the sample"""
    # initialize the lowest distance to the maximum distance between points in the space [-.5,.5]x [-.5,.5]y
    lowest = 2 ** .5
    for node in nodes:
        distance = ((samp.x - node.x) ** 2 + (samp.y + node.y) ** 2) ** .5
        if distance < lowest:
            lowest = distance
            nearest_node = node
    return nearest_node


def collision_free(node_1, node_2, obstacles):
    """Returns true if a collision is detected. This uses the intersection finding approach described here:
    http://mathworld.wolfram.com/Circle-LineIntersection.html"""
    for obstacle in obstacles:
        d_x = node_2.x - node_1.x
        d_y = node_2.y - node_1.y
        d_r = (d_x ** 2 + d_y ** 2) ** .5
        D = node_1.x * node_2.y - node_2.x * node_1.y
        discriminant = obstacle[2] ** 2 * d_r ** 2 - D ** 2

        # check if intersection occurs on an infinite line:
        if discriminant >= 0:
            # calculate the locations of the intersections. If the line is tangent to the circle x_1 = x_2 and y_1 = y_2
            x_1 = D * d_y + sgn(d_y) * d_x * discriminant ** .5 / (d_r ** 2 )
            y_1 = -D * d_x + abs(d_y) * discriminant ** .5 / (d_r ** 2)
            x_2 = D * d_y - sgn(d_y) * d_x * discriminant ** .5 / (d_r ** 2 )
            y_2 = -D * d_x - abs(d_y) * discriminant ** .5 / (d_r ** 2)

            # check if if the intersection points fall between the end points of the line segment
            # if the line is not vertical (node_1.x is not equal to node_2.x we check if either x value is between the
            # end points, if it is vertical we check y instead
            if node_1.x != node_2.x:
                # check if at least one of the intersections is between the endpoints
                if bound(x_1, x_2, node_1.x, node_2.x):
                    return False
            elif bound(y_1, y_2, node_1.y, node_2.y):
                return False
    return True


def bound(v_1, v_2, v_3, v_4):
    """Checks if one or both of the two values are between the other two"""
    # get a sorted list of values
    coordinate_list = sorted([v_1, v_2, v_3, v_4])
    # take the middle two
    coordinate_list = coordinate_list[1:3]
    # if one or both of the intersections are in the middle two values it must be between the endpoints
    if (v_1 in coordinate_list) or (v_2 in coordinate_list):
        return True


def sgn(value):
    """Used in the collision detection calculations. See the MathWorld link in the collision free docstring"""
    if value < 0:
        return -1
    else:
        return 1


# import obstacles.csv
obstacles = []
with open('obstacles.csv') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        # if the row is not a comment create a new node object and add it to the list of nodes
        if not row[0].startswith('#'):
            # add the obstacle to the list of obstacles, and change the diameter to radius
            obstacles.append([float(row[0]), float(row[1]), float(row[2])/2])

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
    # 6: if the motion is collision-free:
    if collision_free(x_samp, x_nearest, obstacles):
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
        print("collision detected")
