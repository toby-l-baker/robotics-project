import numpy as np
import rospy

from controller import BaseController


class PIDController(BaseController):
    def __init__(self):
        super(PIDController, self).__init__()

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
            # the path.
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
        # TODO: INSERT CODE HERE
        # Compute the next control using the PD control strategy.
        # Consult the project report for details on how PD control
        # works.
        #
        # First, compute the cross track error. Then, using known
        # gains, generate the control.
        err_vector = self.get_error(pose, index)
        theta = self.get_reference_pose(index)[2]
        velocity = self.get_reference_pose(index)[3]

        r = np.array([np.array([np.cos(theta), np.sin(theta)]), np.array([-np.sin(theta), np.cos(theta)])])
        ect = np.dot(r, err_vector)[1]

        # d/dt ect = V sin(theta_e)
        deriv = velocity * np.sin(pose[2] - theta)

        # Apply PD coefficients
        delta = -((self.kp * ect) + (self.kd * deriv))

        return np.array([velocity, delta])

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        with self.path_lock:
            pass

    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            self.kp = float(rospy.get_param("/pid/kp", 0.40)) # old: 0.15
            self.kd = float(rospy.get_param("/pid/kd", 0.30))  # old: 0.20
            self.finish_threshold = float(rospy.get_param("/pid/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/pid/exceed_threshold", 4.0))
            # Could use this value to replace our constant offset into the reference list,
            # by picking next closest point to reference pose this distance away.
            # Average distance from the current reference pose to lookahed.
            self.waypoint_lookahead = float(rospy.get_param("/pid/waypoint_lookahead", 0.6))
