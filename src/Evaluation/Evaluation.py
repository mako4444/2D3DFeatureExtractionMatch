import numpy as np
import open3d as o3d
import threading
import time

from ImageVisualizer import ImageVisualizer
from PointCloudVisualizer import PointCloudVisualizer

import os.path as osp
import sys
this_dir = osp.dirname(__file__) # the path of current file
project_root = osp.abspath(osp.join(this_dir, '../')) 
sys.path.append(project_root)
from Utility.LoadDataHelper import *

class Evaluation:
    def __init__(self, image_path, pcd_path):
        self.is_done = False
        
        self.pcd_path = pcd_path
        self.pcd = None
        self.pcd_vis = None

        self.image_path = image_path
        self.image = None
        self.image_vis = None
    
    def run(self):
        self.image = read_from_image_file(self.image_path)
        self.pcd = read_from_pcd_file(self.pcd_path)

        app = o3d.visualization.gui.Application.instance
        app.initialize()

        self.pcd_vis = o3d.visualization.O3DVisualizer("Point Cloud")
        self.pcd_vis.set_on_close(self.on_point_cloud_window_closing)

        self.image_vis = o3d.visualization.O3DVisualizer("Image Frame")
        self.image_vis.set_on_close(self.on_image_window_closing)

        app.add_window(self.pcd_vis)
        app.add_window(self.image_vis)

        threading.Thread(target=self.update_thread).start()

        app.run()

    def on_point_cloud_window_closing(self):
        self.is_done = True
        return True  # False would cancel the close

    def on_image_window_closing(self):
        return True

    def update_thread(self):
        # This is NOT the UI thread, need to call post_to_main_thread() to update
        # the scene or any part of the UI.
        bounds = self.pcd.get_axis_aligned_bounding_box()
        extent = bounds.get_extent()

        def add_first_cloud():
            mat = o3d.visualization.rendering.Material()
            mat.shader = "defaultUnlit"
            self.pcd_vis.add_geometry("point cloud map", self.pcd, mat)
            self.pcd_vis.reset_camera_to_default()
            self.pcd_vis.setup_camera(60, bounds.get_center(), bounds.get_center() + [0, 0, -3], [0, -1, 0])

        def add_first_image():
            mat = o3d.visualization.rendering.Material()
            mat.shader = "defaultUnlit"
        #    self.image_vis.add_geometry("Image Frame", self.image, mat)

        o3d.visualization.gui.Application.instance.post_to_main_thread(self.pcd_vis, add_first_cloud)
        o3d.visualization.gui.Application.instance.post_to_main_thread(self.image_vis, add_first_image)

        while not self.is_done:
            time.sleep(0.1)

            # Perturb the cloud with a random walk to simulate an actual read
            pts = np.asarray(self.pcd.points)
            magnitude = 0.005 * extent
            displacement = magnitude * (np.random.random_sample(pts.shape) -
                                        0.5)
            new_pts = pts + displacement
            self.pcd.points = o3d.utility.Vector3dVector(new_pts)

            if self.is_done:  # might have changed while sleeping
                break
            o3d.visualization.gui.Application.instance.post_to_main_thread(
                self.main_vis, self._update_cloud)

    # Note: if the number of points is less than or equal to the
    #       number of points in the original object that was added,
    #       using self.scene.update_geometry() will be faster.
    #       Requires that the point cloud be a t.PointCloud.
    def _update_cloud(self):
        return
        #self.main_vis.remove_geometry(CLOUD_NAME)
        #mat = o3d.visualization.rendering.Material()
        #mat.shader = "defaultUnlit"
        #self.main_vis.add_geometry(CLOUD_NAME, self.cloud, mat)
    
if __name__ == "__main__":
    evaluation = Evaluation(
        "/home/han/Projects/2D3DDataProcessing/Data/Robotcar_radar/2019-01-10-11-46-21_2200-4200/mono_left_frames_1876-3575/1547120895624990.png", 
        "/home/han/Projects/2D3DDataProcessing/Data/Robotcar_radar/2019-01-10-11-46-21_2200-4200/570645.pcd")
    evaluation.run()