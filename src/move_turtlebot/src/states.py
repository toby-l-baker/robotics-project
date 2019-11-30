"""
Classes defining states
"""

import numpy as np

def vector(obj):
    result = []
    attributes = ["x", "y", "z", "w"]
    for attr in attributes:
        if hasattr(obj, attr):
            result.append(getattr(obj, attr))
    return np.array(result)

class State():
    """
    An abstract class for a state machine state value
    """
    def __init__(self, name=None):
        self.name = name

    def entry(self, machine):
        """
        Invoked on entering the state - only once
        machine - reference to state machine

        no return
        """
        pass

    def exit(self, machine):
        """
        Invoked to check if time to exit the state - repeated
        machine - reference to state machine

        returns None or a new state
        """
        return None


class Idle(State):
    """
    State corresponding to robots remaining idle
    """
    def __init__(self):
        super(Idle, self).__init__("Idle")

    def entry(self, machine):
        machine.pub_motors()

    def exit(self, machine):
        return None

class InitialNavigation(State):
    """
    State corresponding to robots navigating to each other
    """
    def __init__(self):
        super(InitialNavigation, self).__init__("InitialNavigation")
        self.proximity = rospy.get_param("~initial_proximity")

    def entry(self, machine):
        pass

    def exit(self, machine):
        my_pose = machine.my_pose
        other_pose = machine.other_pose
        # Check if poses are close to each other
        my_point = vector(my_pose.pose.position)
        other_point = vector(other_pose.pose.position)
        if np.linalg.norm(my_point - other_point) < self.proximity:
            if machine.name == machine.package_owner:
                return FollowerAlign()
            else:
                return LeaderAlign()

        return None

class FollowerAlign(State):
    """
    State corresponding to robot following to align
    """
    def __init__(self):
        super(FollowerAlign, self).__init__("FollowerAlign")

class LeaderAlign(State):
    """
    State corresponding to robot leading to align
    """
    def __init__(self):
        super(LeaderAlign, self).__init__("LeaderAlign")

class ExchangePackage(State):
    """
    State corresponding to exchanging package
    """
    def __init__(self)
        super(ExchangePackage, self).__init__("ExchangePackage")

class FinalNavigation(State):
    """
    State corresponding to robots navigating away from each other
    """
    def __init__(self):
        super(FinalNavigation, self).__init__("FinalNavigation")

