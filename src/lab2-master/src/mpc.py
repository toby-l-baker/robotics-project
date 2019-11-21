import numpy as np
import rospy
import utils

from controller import BaseController

from nav_msgs.srv import GetMap

from rosviz import viz_paths_cmap

class ModelPredictiveController(BaseController):
    def __init__(self):
        super(ModelPredictiveController, self).__init__()

        self.reset_params()
        self.reset_state()

    def get_reference_index(self, pose):
        '''
        get_reference_index finds the index i in the controller's path
            to compute the next control control against
        input:
            pose - current pose of the car, represented as [x, y, heading]
        output:
            i - referencence index
        '''
        with self.path_lock:
            # TODO: INSERT CODE HERE
            # Determine a strategy for selecting a reference point
            # in the path. One option is to find the nearest reference
            # point and chose the next point some distance away along
            # the path. You are welcome to use the same method for MPC
            # as you used for PID or Pure Pursuit.
            #
            # Note: this method must be computationally efficient
            # as it is running directly in the tight control loop.

            # array is the L1 distance from pose to each point in the reference path.
            array = np.sum(np.abs(np.subtract(self.path[:,0:2], pose[0:2])), axis=1)
            # index of closest point in reference path.
            closest = np.argmin(array)
            # From closest, can find point waypoint_lookahead forward.
            dists = np.sum(np.abs(np.subtract(self.path[:,0:2], self.path[closest,0:2])), axis=1)
            err_vals = np.abs(np.subtract(dists, self.waypoint_lookahead**2))
            res = np.argmin(err_vals[closest:])
            return min(res + closest, len(self.path) - 1)

    def get_control(self, pose, index):
        '''
        get_control - computes the control action given an index into the
            reference trajectory, and the current pose of the car.
            Note: the output velocity is given in the reference point.
        input:
            pose - the vehicle's current pose [x, y, heading]
            index - an integer corresponding to the reference index into the
                reference path to control against
        output:
            control - [velocity, steering angle]
        '''
        assert len(pose) == 3

        rollouts = np.zeros((self.K, self.T, 3))

        # TODO: INSERT CODE HERE
        # With MPC, you should roll out K control trajectories using
        # the kinematic car model, then score each one with your cost
        # function, and finally select the one with the lowest cost.

        # For each K trial, the first position is at the current position (pose)
        rollouts[:,0,:] = pose
        reference_point = self.get_reference_pose(index)

        # For control trajectories, get the speed from reference point.
        ref_velocity = reference_point[3]
        self.trajs[:,:,0] = ref_velocity

        # Then step forward the K control trajectory T timesteps using
        # self.apply_kinematics

        for t in range(0, self.T - 1):
            # use rollouts[:,t] as positions and self.trajs[:,t] as controls
            # numeric integration done inside apply_kinematics
            rollouts[:,t+1] = self.apply_kinematics(rollouts[:,t], self.trajs[:,t])

        # Apply the cost function to the rolled out poses.
        costs = self.apply_cost(rollouts, index)

        # Finally, find the control trajectory with the minimum cost.
        min_control = np.argmin(costs)

        # Visualize the heatmap of the cost function on each trajectory in rviz
        #viz_paths_cmap(rollouts, costs, scale=0.01)

        # Return the controls which yielded the min cost.
        return self.trajs[min_control][0]

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        with self.path_lock:
            self.trajs = self.get_control_trajectories()
            assert self.trajs.shape == (self.K, self.T, 2)
            self.scaled = np.zeros((self.K * self.T, 3))
            self.bbox_map = np.zeros((self.K * self.T, 2, 4))
            self.perm = np.zeros(self.K * self.T).astype(np.int)
            self.map = self.get_map()
            self.perm_reg = self.load_permissible_region(self.map)
            self.map_x = self.map.info.origin.position.x
            self.map_y = self.map.info.origin.position.y
            self.map_angle = utils.rosquaternion_to_angle(self.map.info.origin.orientation)
            self.map_c = np.cos(self.map_angle)
            self.map_s = np.sin(self.map_angle)

    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            self.wheelbase = float(rospy.get_param("trajgen/wheelbase", 0.33))
            self.min_delta = float(rospy.get_param("trajgen/min_delta", -0.34))
            self.max_delta = float(rospy.get_param("trajgen/max_delta", 0.34))

            self.K = int(rospy.get_param("mpc/K", 150))
            self.T = int(rospy.get_param("mpc/T", 45))

            self.speed = float(rospy.get_param("mpc/speed", 1.0))
            self.finish_threshold = float(rospy.get_param("mpc/finish_threshold", 1.0))
            self.exceed_threshold = float(rospy.get_param("mpc/exceed_threshold", 4.0))
            # Average distance from the current reference pose to lookahed.
            self.waypoint_lookahead = float(rospy.get_param("mpc/waypoint_lookahead", 0.2))
            self.collision_w = float(rospy.get_param("mpc/collision_w", 3e5)) 
            self.error_w = float(rospy.get_param("mpc/error_w", 1.5))

            self.car_length = float(rospy.get_param("mpc/car_length", 0.33))
            self.car_width = float(rospy.get_param("mpc/car_width", 0.30))

            self.trajs = self.get_control_trajectories()

    def get_control_trajectories(self):
        '''
        get_control_trajectories computes K control trajectories to be
            rolled out on each update step. You should only execute this
            function when initializing the state of the controller.

            various methods can be used for generating a sufficient set
            of control trajectories, but we suggest stepping over the
            control space (of steering angles) to create the initial
            set of control trajectories.
        output:
            ctrls - a (K x T x 2) vector which lists K control trajectories
                of length T
        '''
        # TODO: INSERT CODE HERE
        # Create a trajectory library of K trajectories for T timesteps to
        # roll out when computing the best control.
        ctrls = np.zeros((self.K, self.T, 2))
        step_size = (self.max_delta - self.min_delta) / (self.K - 1)
        cur_angle = self.min_delta
        percent = 0.75
        for i in range(self.K):
            ctrls[i,:int(percent * self.T),0] = self.speed
            ctrls[i,:int(percent * self.T),1] = cur_angle
            cur_angle += step_size
        ctrls[:,int(percent * self.T):,0] = self.speed
        ctrls[:,int(percent * self.T):,1] = 0.0
        return ctrls

    def apply_kinematics(self, cur_x, control):
        '''
        apply_kinematics 'steps' forward the pose of the car using
            the kinematic car model for a given set of K controls.
        input:
            cur_x   (K x 3) - current K "poses" of the car
            control (K x 2) - current controls to step forward
        output:
            new_vals  (K x 3) - the new poses after pushing cur_x through
                                the motion model.
        '''
        # TODO: INSERT CODE HERE
        # Use the kinematic car model discussed in class to step
        # forward the pose of the car. We will step all K poses
        # simultaneously, so we recommend using Numpy to compute
        # this operation.
        #
        # ulimately, return a triplet with x_dot, y_dot_, theta_dot
        # where each is a length K numpy vector.
        dt = 0.04
        deltas = np.zeros((self.K, 3))
        # assuming velocity never changes
        velocity = control[0,0]
        if not np.isclose(control[0,1], 0.0):
		# Delta for theta - add to old theta to get new theta.
		theta_delta = np.multiply(dt * velocity / self.car_length, np.tan(control[:,1]))
		# Add old thetas with delta to get theta new
		theta_new = np.add(cur_x[:,2], theta_delta)
		x_delta = np.multiply(self.car_length, np.divide(np.subtract(np.sin(theta_new), np.sin(cur_x[:,2])), np.tan(control[:,1])))
		y_delta = np.multiply(self.car_length, np.divide(np.subtract(np.cos(cur_x[:,2]), np.cos(theta_new)), np.tan(control[:,1])))
		x_new = np.add(cur_x[:,0], x_delta)
		y_new = np.add(cur_x[:,1], y_delta)
		return np.vstack((x_new, y_new, theta_new)).T
        else:
		# change in angle is 0, so theta_new is theta_old
		theta_new = cur_x[:,2]
                x_delta = np.multiply(dt * velocity, np.cos(theta_new))
		x_new = np.add(cur_x[:,0], x_delta)
                y_delta = np.multiply(dt * velocity, np.sin(theta_new))
		y_new = np.add(cur_x[:,1], y_delta)
		return np.vstack((x_new, y_new, theta_new)).T

    def apply_cost(self, poses, index):
        '''
        rollouts (K,T,3) - poses of each rollout
        index    (int)   - reference index in path
        '''
        all_poses = poses.copy()
        all_poses.resize(self.K * self.T, 3)
        collisions = self.check_collisions_in_map(all_poses)
        collisions.resize(self.K, self.T)
        # Possibly weight collisions sooner as worse than collisions later.
        scaled = (np.arange(self.T, 0, -1) * 0.20).astype(int) + 2
        collisions = collisions * scaled
        collision_cost = collisions.sum(axis=1) * self.collision_w
        error_cost = np.linalg.norm(poses[:, self.T - 1, :2] - self.path[index, :2], axis=1) * self.error_w

        return collision_cost + error_cost

    def check_collisions_in_map(self, poses):
        '''
        check_collisions_in_map is a collision checker that determines whether a set of K * T poses
            are in collision with occupied pixels on the map.
        input:
            poses (K * T x 3) - poses to check for collisions on
        output:
            collisions - a (K * T x 1) float vector where 1.0 signifies collision and 0.0 signifies
                no collision for the input pose with corresponding index.
        '''

        self.world2map(poses, out=self.scaled)

        L = self.car_length
        W = self.car_width

        # Specify specs of bounding box
        bbox = np.array([
            [L / 2.0, W / 2.0],
            [L / 2.0, -W / 2.0],
            [-L / 2.0, W / 2.0],
            [-L / 2.0, -W / 2.0]
        ]) / (self.map.info.resolution)

        x = np.tile(bbox[:, 0], (len(poses), 1))
        y = np.tile(bbox[:, 1], (len(poses), 1))

        xs = self.scaled[:, 0]
        ys = self.scaled[:, 1]
        thetas = self.scaled[:, 2]

        c = np.resize(np.cos(thetas), (len(thetas), 1))
        s = np.resize(np.sin(thetas), (len(thetas), 1))

        self.bbox_map[:, 0] = (x * c - y * s) + np.tile(np.resize(xs, (len(xs), 1)), 4)
        self.bbox_map[:, 1] = (x * s + y * c) + np.tile(np.resize(ys, (len(ys), 1)), 4)

        bbox_idx = self.bbox_map.astype(np.int)

        self.perm[:] = 0
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 0], bbox_idx[:, 0, 0]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 1], bbox_idx[:, 0, 1]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 2], bbox_idx[:, 0, 2]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 3], bbox_idx[:, 0, 3]])

        return self.perm.astype(np.float)

    def get_map(self):
        '''
        get_map is a utility function which fetches a map from the map_server
        output:
            map_msg - a GetMap message returned by the mapserver
        '''
        print("Getting map")
        srv_name = rospy.get_param("static_map", default="/static_map")
        rospy.logdebug("Waiting for map service")
        rospy.wait_for_service(srv_name)
        rospy.logdebug("Map service started")

        map_msg = rospy.ServiceProxy(srv_name, GetMap)().map
        print("Got map")
        return map_msg

    def load_permissible_region(self, map):
        '''
        load_permissible_region uses map data to compute a 'permissible_region'
            matrix given the map information. In this matrix, 0 is permissible,
            1 is not.
        input:
            map - GetMap message
        output:
            pr - permissible region matrix
        '''
        map_data = np.array(map.data)
        array_255 = map_data.reshape((map.info.height, map.info.width))
        pr = np.zeros_like(array_255, dtype=bool)

        # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
        # With values 0: not permissible, 1: permissible
        pr[array_255 == 0] = 1
        pr = np.logical_not(pr)  # 0 is permissible, 1 is not

        return pr

    def world2map(self, poses, out):
        '''
        world2map is a utility function which converts poses from global
            'world' coordinates (ROS world frame coordinates) to 'map'
            coordinates, that is pixel frame.
        input:
            poses - set of X input poses
            out - output buffer to load converted poses into
        '''
        out[:] = poses
        # translation
        out[:, 0] -= self.map_x
        out[:, 1] -= self.map_y

        # scale
        out[:, :2] *= (1.0 / float(self.map.info.resolution))

        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(out[:, 0])
        out[:, 0] = self.map_c * out[:, 0] - self.map_s * out[:, 1]
        out[:, 1] = self.map_s * temp + self.map_c * out[:, 1]
        out[:, 2] += self.map_angle
