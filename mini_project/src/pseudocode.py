
class mini_project:
    def __init__(self, args):
        self.start = args.start  # start point information
        self.destination = args.destination  # destination point information
        self.theta_range = args.theta_range  # range/contrain of the angle theta
        self.psi_range = args.psi_range  # range/contrain of the angle psi
        self.omega_range = args.omega_range  # range/contrain of the psi_dot
        self.v_range = v_range  # range/contrain of the velocity
        self.alpha_range = alpha_range  # range/contrain of the theta_dot
        self.acc_range = acc_range  # range of the acceleration.
        self.ball_radius = None  # the radius of the ball of contaning the goal
        self.avoid_plane = args.rope  # the vertical plane to avoid

    def path_check(self, path):
        # check the path for rope interaction
        # get the path the normal way without considering the rope
        path = rrt_goal_bias()
        new_path = []

        # check if the the rope is on the was of our path.
        while path:
            p = path.pop(0)
            if is_rope(p):
                # create a new path based on the description
                # given about the projectile motion, by turing
                # off the yaw control and act as projectile toward goal
                new_path = change_path(p)
                #return remaining of the path + the new path using this position as the starting point
                return path+new_path

    def rrt_goal_bias(self):
        # return the path to goal with the controller input associated
        # get the ball_radius and the angle of the goal points
        self.ball_radius, theta_dest, psi_dest = compute_goal_point()
        # set nodes and edges variables
        nodes = []
        edges = []
        # hold the controller input
        ctrl_input = []
        # add the start point to the node list
        nodes.add(self.start)
        # set counter to  zero
        n = 0
        #################################################################################################
        # get the altitude to start the motion from
        r, theta, psi = get_clear_altitude()
        # get controller input to get there
        theta_dot, psi_dot, a, node = compute_controller_input(
            t_span, (r, theta, psi), nodes)

        # create an edge
        edge = [(node.point), (x, y, z)]

        # make a new node
        new_node = [(r, theta, psi), (x, y, z)]
        # add new node to all nodes
        nodes.add(new_node)
        # add edge to edges
        edges.add(edge)

        # add controle input
        ctrl_input.add((theta_dot, psi_dot, a))
        #############################################################################################
        # compute thw path using rrt goal bias
        while not in goal_ragion:
            # sample a point in the workspace using a polar coodinate
            r, theta, psi = sample_point(n)
            # get a controller input from the sampled point
            theta_dot, psi_dot, a, node = compute_controller_input(
                t_span, (r, theta, psi), nodes)
            # use the controller inputs and state space of the system to find x,y,z
            x, y, z = compute_cordinate((theta_dot, psi_dot, a))
            # make an edge with the new x,y z cordinate
            edge = [node, (x, y, z)]
            # check for edge validity
            if is_obstacle_free(edge):
                # add the edge to the list of valide edge as well as the node
                new_node = [(r, theta, psi), (x, y, z)]
                nodes.add(new_node)
                edges.add(edge)
                ctrl_input.add((theta_dot, psi_dot, a))
            # increment the counter
            n += 1
        # find our path to goal based of the valide edges and nodes
        path = find_path(nodes, edges, ctrl_input)
        # return the path
        return check_path(path)

    def get_clear_altitude(self):
        # sample the work space  in the start area in the z direction to get a
        # clear path for a projectile like motion with no static obstacles
        return r, theta, psi

    def sample_point(self, n):
        # n is used to bias our sampling toward goal
        # sample a point in the workspace
        if n % 90:
            # pick a randome pon along the max radius of the sphere that contains out goal
            r = random(0, ball_radius)
            # pick a ramdom in our theta range
            theta = random(self.theta_range)

            # pick a random point in our psi range
            psi = random(self.psi_range)
            return r, theta, psi  # return the values
        return sample_near_goal

    def compute_goal_point(self):
        # using the destination data we can calculate the radius of the ball in which
        # the destinantion point is located
        # the angles of the destination point from our reference point.
        ball_radius = distance(self.start, self.destination)
        theta_dest, psi_dest = calc_angles(self.destination)

        return ball_radius, theta_dest, psi_dest

    def compute_controller_input(self, t_span, data):
        # calculate the controller input based on the data and time_span
        # we can use the time span to control the velocities and accelation
        # return the differentiation of both the theta_dot and psi_dot calculated as
        # (x_1-x_0)/t_span
        # also return the acceleration and the point chosen for the calculation
        return theta_dot, psi_dot, a, closed_node

    def compute_cordinate(self, data):
        # calculate the x,y,z using the controller.
        # we know that v=rw or r_x*w for example

        q[i+1] = A*q[i] + Bu[i]
        y = C*q[i]
        return y

    def is_obstacle_free(self, edge):
        # check if our new edge is obstacle free
        # because the drone is not a point we have to consider the edge as a 3d box or tunnel
        # also check if edge goes through vertical plane
        if obstacle_free:
            return true
        return false


if __name__ == "__main__":
    mini = mini_project(args)
    path = mini.rrt_goal_bias()
