import numpy as np
import open3d as o3d

import os.path as osp
import sys
this_dir = osp.dirname(__file__) # the path of current file
project_root = osp.abspath(osp.join(this_dir, '../')) 
sys.path.append(project_root)
from Utility.LoadDataHelper import *

class PointCloudVisualizer:
    def __init__(self, pcd):
        self.pcd = pcd
        self.visualizer = o3d.visualization.Visualizer()
        self.visualizer.create_window()
        self.visualizer.add_geometry(self.pcd)

if __name__ == "__main__":
    pcd = read_from_pcd_file("/home/han/Projects/2D3DDataProcessing/Data/Robotcar_radar/2019-01-10-11-46-21_2200-4200/570645.pcd")
    pcd_visualizer = PointCloudVisualizer(pcd)
    pcd_visualizer.show()