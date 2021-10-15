import numpy as np
import open3d as o3d

import os.path as osp
import sys
this_dir = osp.dirname(__file__) # the path of current file
project_root = osp.abspath(osp.join(this_dir, '../')) 
sys.path.append(project_root)
from Utility.LoadDataHelper import *

class ImageVisualizer:
    def __init__(self, image):
        self.image = image
        self.visualizer = o3d.visualization.Visualizer()
        self.visualizer.create_window()
        self.visualizer.add_geometry(self.image)
        
if __name__ == "__main__":
    image = read_from_image_file("/home/han/Projects/2D3DDataProcessing/Data/Robotcar_radar/2019-01-10-11-46-21_2200-4200/mono_left_frames_1876-3575/1547120895624990.png")
    image_visualizer = ImageVisualizer(image)
    image_visualizer.show()