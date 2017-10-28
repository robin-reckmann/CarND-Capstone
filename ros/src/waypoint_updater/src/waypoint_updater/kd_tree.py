from scipy import spatial
import numpy as np

class KDTree(object):
    def __init__(self):

        self.tree = None
        self.is_initialized = False
        self.x = []
        self.y = []
        self.z = []

    def init_tree(self, waypoints):

        for waypoint in waypoints:
            self.x.append(waypoint.pose.pose.position.x)
            self.y.append(waypoint.pose.pose.position.y)
            self.z.append(waypoint.pose.pose.position.z)

        data = zip(self.x,self.y,self.z)

        self.tree = spatial.KDTree(data)

        self.is_initialized = True

    def query(self, current_pose):

        distance, wps_closest = self.tree.query(
            np.array([[current_pose.position.x, current_pose.position.y, current_pose.position.z]]))

        return distance[0], wps_closest[0]