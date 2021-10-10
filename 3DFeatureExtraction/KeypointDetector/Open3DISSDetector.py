import numpy as np
import open3d as o3d
import copy
import os.path as osp
import sys
this_dir = osp.dirname(__file__) # the path of current file
project_root = osp.abspath(osp.join(this_dir, '../..')) 
sys.path.append(project_root)
from Utility.LoadPointCloud import read_from_pcd_file
from Utility.Visualizer import *

class Open3DISSDetector:
    def __init__(self, pcd):
        self.pcd = pcd
        
    def detect_ISS(self, salient_radius=0.005, non_max_radius=0.005, gamma_21=0.5, gamma_32=0.5):
        keypoints_cloud = o3d.geometry.keypoint.compute_iss_keypoints(self.pcd, salient_radius, non_max_radius, gamma_21, gamma_32)
        return keypoints_cloud

if __name__ == "__main__":
    pcd = read_from_pcd_file("/home/han/Projects/2D3DDataProcessing/Data/Robotcar_radar/Scenes/114793.pcd")
    iss_detector = Open3DISSDetector(pcd)
    keypoints_cloud = iss_detector.detect_ISS(salient_radius=0.5, non_max_radius=0.4, gamma_21=0.3, gamma_32=0.15)

    visualize_kp_in_pcd(keypoints_cloud, pcd)
    #visualize_kp_only(keypoints_cloud)