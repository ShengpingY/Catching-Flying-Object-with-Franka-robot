import numpy as np

class CalculateTransformMatrix:
    def __init__(self):
        # Initialize lists to store points in both coordinate systems
        self.points_A = []
        self.points_B = []

        # Transformation parameters
        self.m_ratio = 0.0
        self.m_translation = np.array([0.0, 0.0, 0.0])
        self.m_rotation = np.eye(3)

    def add_point_pair(self, point_A, point_B):
        # Add point pairs to the lists
        self.points_A.append(point_A)
        self.points_B.append(point_B)

    def calibrate(self):
        # Convert lists to numpy arrays
        points_A = np.array(self.points_A)
        points_B = np.array(self.points_B)

        # Compute scale ratio
        # scale_ratios = np.linalg.norm(points_B - np.roll(points_B, 1, axis=0), axis=1) / \
        #                np.linalg.norm(points_A - np.roll(points_A, 1, axis=0), axis=1)
        # self.m_ratio = np.mean(scale_ratios)

        # Compute rotation matrix using Singular Value Decomposition (SVD)
        centroid_A = np.mean(points_A, axis=0)
        centroid_B = np.mean(points_B, axis=0)
        H = np.dot((points_A - centroid_A).T, (points_B - centroid_B))
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)

        # Handle reflections
        if np.linalg.det(R) < 0:
            Vt[2,:] *= -1
            R = np.dot(Vt.T, U.T)

        self.m_rotation = R

        # Compute translation vector
        self.m_translation = centroid_B - np.dot(centroid_A, self.m_rotation)

    def get_transformmatrix(self):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = self.m_rotation
        transform_matrix[:3, 3] = self.m_translation
        return transform_matrix

