import numpy as np
import rospy
from controller import BaseController


class PurePursuitController(BaseController):
    def __init__(self):
        super(PurePursuitController, self).__init__()

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
            # Use the pure pursuit lookahead method described in the
            # handout for determining the reference index.
            #
            # Note: this method must be computationally efficient
            # as it is running directly in the tight control loop.

            dists = np.sum(np.abs(np.subtract(self.path[:,0:2], pose[0:2])), axis=1)
            closest = np.argmin(dists)
            err_vals = np.abs(np.subtract(dists, self.pose_lookahead**2))
            res = np.argmin(err_vals[closest:])
            #print("{0} - dist {1} - lookahead {2}".format(res, err_vals[res], self.pose_lookahead**2))
            #print("closest {0}".format(closest))
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
        # TODO: INSERT CODE HERE
        # First, compute the appropriate error.
        #
        # Then, use the pure pursuit control method to compute the
        # steering angle. Refer to the hand out and referenced
        # articles for more details about this strategy.
        e_p = self.get_error(pose, index)

        alpha = np.arctan2(-e_p[1], -e_p[0]) - pose[2]

        u = np.arctan2(2 * self.car_length * np.sin(alpha), self.pose_lookahead)

        velocity = self.get_reference_pose(index)[3]
        return np.array([velocity, u])

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        pass

    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            self.speed = float(rospy.get_param("/pid/speed", 1.0))
            self.finish_threshold = float(rospy.get_param("/pid/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/pid/exceed_threshold", 4.0))
            # Lookahead distance from current pose to next waypoint. Different from
            # waypoint_lookahead in the other controllers, as those are distance from
            # the reference point.
            self.pose_lookahead = float(rospy.get_param("/purepursuit/pose_lookahead", 0.9))
            self.car_length = float(rospy.get_param("/car_kinematics/car_length", 0.33)) # The length of the car
