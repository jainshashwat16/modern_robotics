from anytree import NodeMixin
import math


class Node(NodeMixin):
    """Data class that stores node coordinates and parents. Subclasses the anytree NodeMixin class to represent the
    tree"""
    count = 0
    list = []
    goal = None

    def __init__(self, x, y, parent=None, start=False, goal=False):
        super(Node, self).__init__()
        self.x = x
        self.y = y
        self.parent = parent
        Node.count += 1
        Node.list.append(self)
        self.node_id = Node.count
        self.open = True
        if start:
            self.past_cost = 0
            self.optimistic_ctg = self.cost_to_go(Node.goal)
            self.est_tot_cost = self.optimistic_ctg
        elif goal:
            Node.goal = self
            self.optimistic_ctg = self.cost_to_go(Node.goal)
            self.past_cost = math.inf
            self.est_tot_cost = math.inf

        else:
            self.optimistic_ctg = self.cost_to_go(Node.goal)
            self.past_cost = math.inf
            self.est_tot_cost = math.inf

    def __repr__(self):
        """Returns a string representation of the node."""
        return f"{self.node_id},{self.x},{self.y},{self.optimistic_ctg}"

    def cost_to_go(self, goal):
        """Returns the straight line distance from the node to the goal."""
        return node_distance(self, goal)

    def nearest_neighbors(self, no_neighbors):
        """Returns the nearest N nodes to the sample."""
        nearest_nodes = {}
        for node in Node.list:
            if node != self:
                distance = node_distance(self, node)
                if len(nearest_nodes) < no_neighbors:
                    nearest_nodes[distance] = node
                else:
                    if distance < max(nearest_nodes):
                        nearest_nodes[distance] = node
                        nearest_nodes.pop(max(nearest_nodes))
        return list(nearest_nodes.values())

    def update(self, parent, cost):
        """If the new cost of traveling to this node is lower than the previous cost update the cost and parent of the
        node"""
        if cost < self.past_cost:
            if self.open:
                self.past_cost = cost
                self.est_tot_cost = self.optimistic_ctg + self.past_cost
                self.parent = parent
                # insert the node in the correct location, by estimated total cost, in open nodes
                Node.list = sorted(Node.list, key=lambda x: x.est_tot_cost)

    def explore(self, edges):
        """Looks through the list of edges for edges connected to the node.
        Runs the update function on the connected node"""
        for edge in edges:
            if edge.id1 == self.node_id:
                Node.list[edge.id2-1].update(self.node_id, self.past_cost + edge.cost)
            elif edge.id2 == self.node_id:
                Node.list[edge.id1-1].update(self.node_id, self.past_cost + edge.cost)
        # once all edges connected to the node have been explored we close it
        self.open = False


class Edge:
    """Data class for representing edges"""
    def __init__(self, node_1, node_2):
        self.node_1 = node_1
        self.node_2 = node_2
        self.cost = node_distance(node_1, node_2)

    def __repr__(self):
        return f"{self.node_1.node_id},{self.node_2.node_id},{self.cost}"

    def collision_free(self, obstacles):
        """Returns true if no collision is detected. This uses the intersection finding approach described here:
        http://mathworld.wolfram.com/Circle-LineIntersection.html"""
        for obstacle in obstacles:
            n1_x = self.node_1.x - obstacle[0]
            n1_y = self.node_1.y - obstacle[1]
            n2_x = self.node_2.x - obstacle[0]
            n2_y = self.node_2.y - obstacle[1]

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


def node_distance(node_1, node_2):
    """Returns the straight line distance between two nodes."""
    return ((node_1.x - node_2.x) ** 2 + (node_1.y - node_2.y) ** 2) ** .5


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
