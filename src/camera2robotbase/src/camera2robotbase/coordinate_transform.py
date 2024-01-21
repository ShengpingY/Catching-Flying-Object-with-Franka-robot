#!/usr/bin/env python

import numpy as np
import random 
 
class CoordinateConversion(object):
  
  def convert(self, frame_coordinates):
    
    raise NotImplementedError
 
 
class CameraToRobotBaseConversion(CoordinateConversion):
 
  def __init__(self,rot_angle,x_disp,y_disp,z_disp):

    self.angle = np.deg2rad(rot_angle)
    self.X = x_disp
    self.Y = y_disp
    self.Z = z_disp
 
  def convert(self, frame_coordinates):
    """ 
    input: frame_coordinates: coordinate in camera (x, y, z, 1) 
    return: new_frame_coordinates:coordinate in robot_base (x, y, z, 1)     
    """
    
    rot_mat_0_c = np.array([[1, 0, 0],
                            [0, np.cos(self.angle), -np.sin(self.angle)],
                            [0, np.sin(self.angle), np.cos(self.angle)]])
 
    
    disp_vec_0_c = np.array([[self.X],
                             [self.Y], 
                             [self.Z]])
 
    extra_row_homgen = np.array([[0, 0, 0, 1]])

    homgen_0_c = np.concatenate((rot_mat_0_c, disp_vec_0_c), axis=1) 
    homgen_0_c = np.concatenate((homgen_0_c, extra_row_homgen), axis=0) 
 
    new_frame_coordinates = homgen_0_c @ frame_coordinates
 
    return new_frame_coordinates
  
"""
# 以下为测试代码
def main():

  x_disp = -17.8
  y_disp = 24.4
  z_disp = 0.0
  rot_angle = 180    
 
  cam_to_robo = CameraToRobotBaseConversion(rot_angle, x_disp, y_disp, z_disp)
 
 
  print(f'Detecting an object for 3 seconds')
  dt = 0.1     # 定义时间间隔
  t = 0        # 设置开始时间
  while t<3:
    t = t + dt
 
    x = random.randint(250,450)   
    y = random.randint(250,450)
    # z = random.randint(250,450)     
 
    cam_ref_coord = np.array([[x],
                              [y],
                              [0.0],
                              [1]])

    robot_base_frame_coord = cam_to_robo.convert(cam_ref_coord) 
    
    text = "x: " + str(robot_base_frame_coord[0][0]) + ", y: " + str(robot_base_frame_coord[1][0])
    print(f'{t}:{text}') 
 
if __name__ == '__main__':
  main()
 """