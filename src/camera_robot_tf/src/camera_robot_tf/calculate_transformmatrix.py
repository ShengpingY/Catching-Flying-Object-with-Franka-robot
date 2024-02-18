import numpy as np

#已知3个点分别在坐标系A和坐标系B中的坐标，求坐标系B相对于坐标系A的旋转矩阵和平移矩阵

class CalculateTransformMatrix:
    def __init__(self):
        self.point_A1 = np.array([0.0, 0.0, 0.0])
        self.point_A2 = np.array([0.0, 0.0, 0.0])
        self.point_A3 = np.array([0.0, 0.0, 0.0])

        self.point_B1 = np.array([0.0, 0.0, 0.0])
        self.point_B2 = np.array([0.0, 0.0, 0.0])
        self.point_B3 = np.array([0.0, 0.0, 0.0])

        self.m_ratio = 0.0
        self.m_translation = np.array([0.0, 0.0, 0.0])
        self.m_rotation = np.eye(3)

    def cali_ratio(self):
        ratio1 = np.linalg.norm(self.point_B1 - self.point_B2) / np.linalg.norm(self.point_A1 - self.point_A2)
        ratio2 = np.linalg.norm(self.point_B1 - self.point_B3) / np.linalg.norm(self.point_A1 - self.point_A3)
        ratio3 = np.linalg.norm(self.point_B2 - self.point_B3) / np.linalg.norm(self.point_A2 - self.point_A3)
        self.m_ratio = (ratio1 + ratio2 + ratio3) / 3

    def cali_rotation(self):
        vector_A1 = self.point_A2 - self.point_A1
        vector_A2 = self.point_A3 - self.point_A1
        vector_A3 = np.cross(vector_A1, vector_A2)

        vector_B1 = self.point_B2 - self.point_B1
        vector_B2 = self.point_B3 - self.point_B1
        vector_B3 = np.cross(vector_B1, vector_B2)

        self.m_rotation = np.array([vector_B1, vector_B2, vector_B3]).T @ np.linalg.inv(np.array([vector_A1, vector_A2, vector_A3]).T) 

    def cali_translation(self):
        self.m_translation = self.point_B1 -  self.m_rotation @ self.point_A1

    def get_transformmatrix(self):
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = self.m_rotation
        transform_matrix[:3, 3] = self.m_translation
        return transform_matrix

    # def get_transformmatrix(self):
    #     transform_matrix = np.eye(4)
    #     transform_matrix[:3, :3] = self.m_ratio * self.m_rotation
    #     transform_matrix[:3, 3] = self.m_translation
    #     return transform_matrix
    
    # def get_transformmatrix(self):
    #     return self.m_ratio, self.m_rotation, self.m_translation
