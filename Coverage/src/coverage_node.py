import sys, rospy
import math
from nav_msgs.srv import GetMap
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_from_euler

current_robot_row = current_robot_col = 0
sys.setrecursionlimit(10 ** 8)
rows = col = 0
start_x_point = start_y_point = 0
map_reso = 0


class Place:
    LEFT_UP = 0
    RIGHT_UP = 1
    LEFT_DOWN = 2
    RIGHT_DOWN = 3

class Diraction:
    DOWN = "down"
    LEFT = "left"
    RIGHT = "right"
    UP = "up"


class DFSReader:
    INDEX = 0
    PREV_NODE = 1
    DIRECTION = 2


# this class represent a point which have x and y and if point is an obsacle
class Point(object):
    def __init__(self, x, y):
        self.is_obstacle = False
        self.x = x
        self.y = y


    # This function check if the given point is the same point
    def equals_to(self, point):
        return self.x == point.x and self.y == point.y

    # this function set the point as an obstacle
    def set_obstacle(self):
        self.is_obstacle = True

    def shift(self, new_x, new_y):
        self.x += new_x
        self.y += new_y

    # This function make the point to a string
    def __str__(self):
        return "[{},{}] ".format(self.x, self.y)


# The class represents Hamilton's circle and knows how to find such a circle in the graph.
class HamiltonCycle(object):
    def __init__(self, start):
        self.path = []
        self.goal = start

    # this function returns the cycle that was found
    def get_cycle(self):
        return self.path

    # this function save in file the path of the hamilton cycle
    def save_path(self):
        f = open("Coverage_path.txt", "w+")
        for point in self.path:
            f.write("row:{}\tcol:{}\n".format(point.x, point.y))
        f.close()

    #  this function return a list of the nodes that wad not visited yet
    def remove_visited_nodes(self, nodes_list):
        return [node for node in nodes_list if not node.is_visited()]


    # The function gets a limb and chooses which of its neighbors to keep going
    def choose_next_node(self, node, dfs):
        node_edges = self.remove_visited_nodes(node.get_adjs_nodes())
        number_of_edges = len(node_edges)

        # If there is nowhere else to go, we have reached the goal
        if not number_of_edges:
            return None
        #  If there is only one way to go - we will go there
        if number_of_edges == 1:
            return node_edges.pop()
        # If one of the neighbors is from the same vertex in 4DGrid we will move to it
        for adj in node_edges:
            if adj.node_4dgrid.equals_to(node.node_4dgrid):
                return adj

        # take the index in the dfs list
        node_dfs_idx = dfs[node.node_4dgrid][DFSReader.INDEX]
        # If one of the neighbors is a vertex followed by dfs move to it
        for adj in node_edges:
            adj_dfs_idx = dfs[adj.node_4dgrid][DFSReader.INDEX]
            if node_dfs_idx + 1 == adj_dfs_idx:
                return adj
        # else go to the previous node in the dfs
        for adj in node_edges:
            adj_dfs_idx = dfs[adj.node_4dgrid][DFSReader.INDEX]
            if adj_dfs_idx == node_dfs_idx - 1:
                return adj

        first = node_edges[0]
        second = node_edges[1]
        # else go to the neighbor that have the bigger index in the dfs
        return first if dfs[first.node_4dgrid][DFSReader.INDEX] > dfs[second.node_4dgrid][
            DFSReader.INDEX] else second


    # This function check if the given node is the goal node
    def is_goal(self, node):
        return self.goal.equals_to(node)


    #  The function finds Hamilton's circle in the graph
    def find_cycle(self, node, prev_node, first_node, dfs_dict):
        # visit the node and add him to the path
        node.set_visited()
        self.path.append(node.point)
        # the first node doesnt have prev
        if not first_node:
            node.remove_prev_edge(prev_node)
        # choose the next node in the circle
        next_node = self.choose_next_node(node, dfs_dict)
        # if next node is none - it is the goal - finish
        if next_node:
            self.find_cycle(next_node, node, False, dfs_dict)
        else:
            self.path.append(self.goal.point)

    # this function prints the path of the circle to the screen
    def print_path(self):
        for point in self.path:
            print(str(point))


# This class represent a node in the 4d grid
class D4gridNode(object):
    def __init__(self, point):
        self.point = point
        self.adjs = None
        self.up = None
        self.down = None
        self.right = None
        self.left = None
        self.nodes_in_dgrid = []
        self.flag = False

    # This function set the node as an obstacle
    def set_obstacle(self):
        self.point.set_obstacle()

    # this function returns true if the node is an obstacle
    def is_obstacle(self):
        return self.point.is_obstacle

    # this function set the right node
    def set_right(self, other_node):
        if not other_node.is_obstacle():
            self.right = other_node
        self.flag = True

        # this function returns true if the other node same as this node
    def equals_to(self, other_node):
        return self.point.equals_to(other_node.point)

    # this function set the left node
    def set_left(self, other_node):
        if not other_node.is_obstacle():
            self.left = other_node
        self.flag = True

    # this function set the up node
    def set_up(self, other_node):
        if not other_node.is_obstacle():
            self.up = other_node
        self.flag = True

        # this function set the down node
    def set_down(self, other_node):
        if not other_node.is_obstacle():
            self.down = other_node
        self.flag = True

        # This function make a list from all the adj nodes
    def set_adjs(self):
        self.adjs = [[self.right, Diraction.RIGHT], [self.up, Diraction.UP], [self.left, Diraction.LEFT],
                     [self.down, Diraction.DOWN]]

    # this function return a list of the adj
    def get_adjs_nodes(self):
        return [node for node in self.adjs if node[0]]

    # this function connect the node to a node in the d grid
    def add_dgrid_node(self, node):
        self.nodes_in_dgrid.append(node)


#  This class represent a node in the D grid
class DgridNode(object):
    def __init__(self, point):
        self.point = point
        self.up = None
        self.down = None
        self.right = None
        self.left = None
        self.node_4dgrid = None
        self.node_to_direction_dict = {}
        self.visited = False
        self.dict_size = 0

    # This function set the node as an obstacle
    def is_obstacle(self):
        return self.point.is_obstacle

    # this function return true if the node is visited
    def is_visited(self):
        return self.visited

    # This function check if the other node is the same as this node
    def equals_to(self, other):
        return self.point.equals_to(other.point)

    # this function set the node a a visited node
    def set_visited(self):
        self.visited = True

    # This function set a dictionary for the adj and therie direction
    def set_dict(self):
        if self.right:
            self.node_to_direction_dict[self.right] = Diraction.RIGHT
            self.dict_size += 1

        if self.up:
            self.node_to_direction_dict[self.up] = Diraction.UP
            self.dict_size += 1

        if self.left:
            self.node_to_direction_dict[self.left] = Diraction.LEFT
            self.dict_size += 1

        if self.down:
            self.node_to_direction_dict[self.down] = Diraction.DOWN
            self.dict_size += 1

    # This function remove the edge of the prev node
    def remove_prev_edge(self, prev_node):
        direction = prev_node.node_to_direction_dict[self]
        if direction == Diraction.DOWN:
            self.up = None
            return
        if direction == Diraction.UP:
            self.down = None
            return
        if direction == Diraction.LEFT:
            self.right = None
            return
        if direction == Diraction.RIGHT:
            self.left = None

    # this function return a list of the adj
    def get_adjs_nodes(self):
        adjs = [self.right, self.up, self.left, self.down]
        return [node for node in adjs if node]

    # this function connect the node to a node in the d4 grid
    def set_grid4d_node(self, node):
        self.node_4dgrid = node

    # this function set the node as an obstacke
    def set_obstacle(self):
        self.point.set_obstacle()

    # this function set the right node
    def set_right(self, next_node):
        if not next_node.is_obstacle():
            self.right = next_node
        self.dict_size -= 1

    # this function set the left node
    def set_left(self, next_node):
        if not next_node.is_obstacle():
            self.left = next_node
        self.dict_size -= 1

    # this function set the up node
    def set_up(self, next_node):
        if not next_node.is_obstacle():
            self.up = next_node
        self.dict_size -= 1

    # this function set the down node
    def set_down(self, next_node):
        if not next_node.is_obstacle():
            self.down = next_node
        self.dict_size -= 1

    # This function make the point to a string
    def __str__(self):
        return str(self.point)


# This class represent a map
class Map(object):
    def __init__(self, rev_grid, kind):
        self.vertex_matrix = []
        self.dfs_path = []
        self.dfs_dict = {}
        self.create_nodes(rev_grid, kind)
        self.bool = False

    # this function create all the node in the graph
    def create_nodes(self, grid, kind):
        for i, line in enumerate(grid):
            nodes = []
            for j, is_obstacle in enumerate(line):
                p = Point(i, j)
                # crate a node of the given kind
                new_node = D4gridNode(p) if kind == "4d" else DgridNode(p)
                # set the node as and obstacle
                if is_obstacle:
                    new_node.set_obstacle()
                nodes.append(new_node)
            self.vertex_matrix.append(nodes)

    # This function set a dictinary for all the nodes
    def set_dicts_for_all(self):
        for row in self.vertex_matrix:
            for node in row:
                node.set_dict()

    # This function set for each node his adj
    def set_frame(self):
        for i, line in enumerate(self.vertex_matrix):
            for j, node in enumerate(line):
                if not node.is_obstacle():
                    if j != 0:
                        node.set_left(self.vertex_matrix[i][j - 1])
                    if j != len(self.vertex_matrix[0]) - 1:
                        node.set_right(self.vertex_matrix[i][j + 1])
                    if i != 0:
                        node.set_up(self.vertex_matrix[i - 1][j])
                    if i != len(self.vertex_matrix) - 1:
                        node.set_down(self.vertex_matrix[i + 1][j])

    # This function set the adj for all the nodes
    def set_adjs_for_all(self):
        for row in self.vertex_matrix:
            for node in row:
                if not node.is_obstacle():
                    node.set_adjs()

    # this function calculate the dfs
    def dfs_function(self, first_time, node, prev_node, prev_info):
        if first_time:
            self.set_adjs_for_all()

        # add the point to the path
        self.dfs_path.append(node)
        # add the the dict {node :  index in dfs}
        self.dfs_dict[node] = [len(self.dfs_path) - 1, prev_node, prev_info]
        # visit his adj
        for adj in node.get_adjs_nodes():
            if adj[0] not in self.dfs_dict:
                self.dfs_function(False, adj[0], node, adj[1])

    # this function connect the graph to the given graph
    def connect_to(self, d_graph):
        count = 2
        lines = len(self.vertex_matrix)
        nodes_in_line = len(self.vertex_matrix[0])
        for i in range(lines):
            k = count * i
            for j in range(nodes_in_line):
                l = count * j
                node_in_4d = self.vertex_matrix[i][j]
                nodes_in_d = [d_graph.vertex_matrix[k][l], d_graph.vertex_matrix[k][l + 1], d_graph.vertex_matrix[k + 1][l], d_graph.vertex_matrix[k + 1][l + 1]]

                for node in nodes_in_d:
                    node_in_4d.add_dgrid_node(node)
                    node.set_grid4d_node(node_in_4d)
                    if node_in_4d.is_obstacle():
                        node.set_obstacle()
        d_graph.set_frame()

    # This function return the value of those vars
    def nodes_dict(self, prev_node, node):
        left_up_p = prev_node.nodes_in_dgrid[Place.LEFT_UP]
        right_up_p = prev_node.nodes_in_dgrid[Place.RIGHT_UP]
        left_down_p = prev_node.nodes_in_dgrid[Place.LEFT_DOWN]
        right_down_p = prev_node.nodes_in_dgrid[Place.RIGHT_DOWN]
        left_up_n = node.nodes_in_dgrid[Place.LEFT_UP]
        right_up_n = node.nodes_in_dgrid[Place.RIGHT_UP]
        left_down_n = node.nodes_in_dgrid[Place.LEFT_DOWN]
        right_down_n = node.nodes_in_dgrid[Place.RIGHT_DOWN]
        return left_up_p, right_up_p, left_down_p, right_down_p, left_up_n, right_up_n, left_down_n, right_down_n


    # after the dfs this function make the spanning tree walls as an obstacle
    def remove_spanning_tree_edges(self, graph_4d):
        self.bool = True
        # for each node - remove the adj (not to the first)
        for node in graph_4d.dfs_path[1:]:
            # take the prev node from the dict
            prev_node = graph_4d.dfs_dict[node][DFSReader.PREV_NODE]
            # take the direction from the prev
            node_from = graph_4d.dfs_dict[node][DFSReader.DIRECTION]
            # take the directions
            p1, p2, p3, p4, n1, n2, n3, n4 = self.nodes_dict(prev_node, node)
            # remove the adj
            if node_from == Diraction.UP:
                n3.right = n4.left = p1.right = p2.left = None
            if node_from == Diraction.RIGHT:
                n1.down = n3.up = p2.down = p4.up = None
            if node_from == Diraction.DOWN:
                n1.right = n2.left = p3.right = p4.left = None
            if node_from == Diraction.LEFT:
                n2.down = n4.up = p1.down = p3.up = None


# This function calculate the first location in the map
def get_point_on_map():
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
    try:
        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x = trans[0]
        y = trans[1]
        return x, y
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Service call failed: %s" % e)


# This function convert the current point to a point in the grid
def get_point_on_grid():
    global start_y_point, map_reso, rows, current_robot_col, current_robot_row
    robot_x, robot_y = get_point_on_map()
    start_y_point += map_reso * rows
    current_robot_row = math.floor(math.fabs(robot_y - start_y_point) / map_reso)
    current_robot_col = math.floor(math.fabs(robot_x - start_x_point) / map_reso)
    if current_robot_row < 0 or current_robot_row >= rows or current_robot_col < 0 or current_robot_col >= col:
        rospy.logerr("Robot out of bounds")
    else:
        print("Robot found in: " + str(current_robot_row) + ", " + str(current_robot_col))


# returns the difference in absolute value between the 2 y's and 2 x's.
    def get_diff_x_y(x_goal_position, y_goal_position,  current_x_pos, current_y_pos):
        diff_x = math.fabs(x_goal_position - current_x_pos)
        diff_y = math.fabs(y_goal_position - current_y_pos)
        return diff_x, diff_y


# This function check is the position is valid
def is_point_ok(i, j, rows_len, cols_len):
    flag1 = False
    flag2 = False
    if 0 <= i < rows_len:
        flag1 = True
    if 0 <= j < cols_len:
        flag2 = True
    return True if flag1 and flag2 else False


# This function check if the node on the grid
def is_point_on_grid(cur_grid, node):
    value = False
    # if we find the same point - as the node's point - return true
    for v in cur_grid:
        if v.x == node.x and v.y == node.y:
            value = True
            break
    return value


# THis function check the value
def check_the_given_pos(source, row, col, res):
    res = int(res)
    row_in_map = int(res * row)
    col_in_map = int(res * col)
    for i in range(res):
        line_number = i + row_in_map
        for r in range(res):
            if source[line_number][col_in_map + r]:
                return True
    return False


# this function transform the robot location
def convert_location(src, my_reso):
    global current_robot_row, current_robot_col
    values = [[-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]
    nodes_list = []
    visited_points = []
    p_cur = Point(current_robot_row / my_reso, current_robot_col / my_reso)
    nodes_list.append(p_cur)
    while nodes_list:
        p_cur = nodes_list.pop(0)
        len_src = len(src)
        len_src_row = len(src[0])
        pos_giv = p_cur.y + p_cur.x
        for cell in src:
            pos_giv += 1
        if int(p_cur.x) < 0 or int(p_cur.x) >= len_src or int(p_cur.y) < 0 or int(p_cur.y) >= len_src_row:
            rospy.logerr("There was an error")
        else:
            if src[int(p_cur.x)][int(p_cur.y)] == False:
                current_robot_row = int(p_cur.x)
                current_robot_col = int(p_cur.y)
                return
            visited_points[:0] = [p_cur]
            for i in range(len(values)):
                p_cur = Point(int(p_cur.x) + values[i][0], int(p_cur.y) + values[i][1])
                if is_point_ok(p_cur.x, p_cur.y, len(src),
                               len(src[0])) and not is_point_on_grid(visited_points,
                                                                     p_cur) and not is_point_on_grid(
                    nodes_list, p_cur):
                    nodes_list.append(p_cur)
    exit(-1)

# returns a twist message with the desired x and z.
    def get_twist_msg(self, x, z):
        move_msg = Twist()
        move_msg.linear.x = x
        move_msg.angular.z = z
        return move_msg

# this function create new grid according to the new dimention and the diven grid
def create_new_grid(dimension, given_grid):
    obs = False
    given_grid.reverse()
    my_grid = given_grid
    given_grid.reverse()
    width = calc_value(my_grid[0], dimension)
    height = calc_value(my_grid, dimension)
    gris_size = len(my_grid)
    first_reduce = [[obs] * width for x in range(gris_size)]
    sec_reduce = [[obs] * width for x in range(height)]

    first_for = len(my_grid)
    sec_for = len(my_grid[0])
    for r in range(first_for):
        dim_reducer = dimension
        obs = False
        j = 0
        for c in range(sec_for):
            dim_reducer -= 1
            obs = True if my_grid[r][c] else obs
            if not dim_reducer:
                dim_reducer = dimension
                first_reduce[r][j] = obs
                obs = False
                j += 1

    first_for = len(first_reduce[0])
    sec_for = len(first_reduce)
    for c in range(first_for):
        i = 0
        dim_reducer = dimension
        obs = False
        for r in range(sec_for):
            dim_reducer -= 1
            obs = True if first_reduce[r][c] else obs
            if not dim_reducer:
                dim_reducer = dimension
                sec_reduce[i][c] = obs
                obs = False
                i += 1
    return sec_reduce


# This function gets the first grid from the gazibo
def get_first_grid():
    global rows, col, start_x_point, start_y_point, map_reso
    rospy.init_node("coverage_node", argv=sys.argv)
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a %d X %d map @ %.3f m/px" % (
            response.map.info.width, response.map.info.height, response.map.info.resolution))
        rows = response.map.info.height
        col = response.map.info.width
        start_x_point = response.map.info.origin.position.x
        start_y_point = response.map.info.origin.position.y
        map_reso = response.map.info.resolution
        creat_occupancy_grid(response.map)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


# This function creates 2 graphs and return them according to the grids
def create_graphs():
    robot_perimeter = 0.35
    new_size = math.ceil(robot_perimeter / map_reso)
    # create the d grid from the grid
    d_grid = create_new_grid(new_size, grid)
    save_grid("new_grid.txt", d_grid)
    # remove the last line and row from the grid
    d_grid = d_grid[:-1]
    size_of = len(d_grid)
    for i in range(size_of):
        d_grid[i] = d_grid[i][:-1]
    convert_location(d_grid, int(new_size))
    # create the 4d grid from the d grid
    d4_grid = create_new_grid(2, d_grid)
    # save_grid("4dGrid.txt", d4_grid)
    d4_grid.reverse()
    d4_grid_graph = Map(d4_grid, "4d")
    d4_grid_graph.set_frame()
    d_grid.reverse()
    # make a new graph - d grid graph
    dgrid_graph = Map(d_grid, "d")
    return dgrid_graph, d4_grid_graph


# This function returns the path of the hamilton cycle on the graph
def get_final_path(graph_4d, graph_d):
    # calculate the dfs path
    graph_4d.dfs_function(True, graph_4d.vertex_matrix[int(current_robot_row / 2)][int(current_robot_col / 2)], None, None)
    graph_d.remove_spanning_tree_edges(graph_4d)
    graph_d.set_dicts_for_all()
    # first node of the robot
    src_node = graph_d.vertex_matrix[current_robot_row][current_robot_col]
    # get the hamilton cycle
    hamilton = HamiltonCycle(src_node)
    hamilton.find_cycle(src_node, None, True, graph_4d.dfs_dict)
    cycle = hamilton.get_cycle()
    # print the path to the screen
    hamilton.print_path()
    # save the path to a file
    hamilton.save_path()
    return hamilton.path


# This function save to a file a grid
def save_grid(file_name, grid):
    f = open(file_name, "w+")
    for row in reversed(grid):
        for value in row:
            is_obstacle = 1 if value else 0
            f.write(str(is_obstacle))
        f.write("\n")
    f.close()


# This function return the calculated value
def calc_value(my_list, dimension):
    size = math.trunc(len(my_list))
    value = int(size / dimension)
    return value


# This function create the first grid
def creat_occupancy_grid(my_map):
    global rows, cols
    rows = my_map.info.height
    cols = my_map.info.width

    # creating the occupancy grid
    global grid
    grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
    for i in xrange(my_map.info.height):
        for j in xrange(my_map.info.width):
            if my_map.data[i * my_map.info.width + j] == 0:
                grid[i][j] = False
            else:
                grid[i][j] = True

# the main function  - send thr robot to coverage the space
def main():
    get_first_grid()
    get_point_on_grid()
    graph_d, graph_4d = create_graphs()
    graph_4d.connect_to(graph_d)
    final_path = get_final_path(graph_4d, graph_d)
    global map_reso, start_x_point, start_y_point
    new_pos_in_map = 0
    past_nodes = []
    # the robot is on this point already
    final_path.pop(0)
    goal = MoveBaseGoal()
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server(rospy.Duration(20.0))
    done = []
    for point in final_path:
        # calculate the next x and y
        row = int(point.y)
        col = int(point.x)

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = start_x_point + row * math.ceil(0.35 / map_reso) * map_reso + 0.35
        goal.target_pose.pose.position.y = start_y_point - col * math.ceil(0.35 / map_reso) * map_reso - 0.35
        value_of_angle = (goal.target_pose.pose.position.y / goal.target_pose.pose.position.x)

        quat = quaternion_from_euler(0, 0, value_of_angle * (math.pi / 180))
        move_msg = Quaternion(*quat)
        goal.target_pose.pose.orientation = move_msg
        client.send_goal(goal)
        client.wait_for_result()
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED and not is_point_on_grid(done, point):
            done[:0] = [point]


if __name__ == '__main__':
    main()
