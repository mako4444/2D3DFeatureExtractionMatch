import numpy as np
import open3d as o3d
import copy
#from LoadPointCloud import ReadPointCloudFromFile_ModelNet40

# np_pts 3 x n
# key_pts 3 x m
def Visualize(np_pts, np_key_pts):
    np_pts = np_pts.transpose()
    np_key_pts = np_key_pts.transpose()

    original_pointcloud = o3d.geometry.PointCloud()
    keypoint_pointcloud = o3d.geometry.PointCloud()

    kp_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    kp_mesh.paint_uniform_color([1, 0, 0])
    keypoints = o3d.geometry.TriangleMesh()
    
    for point in np_key_pts:
        new_kp = copy.deepcopy(kp_mesh).translate((point[0], point[1], point[2]))
        keypoints = keypoints + new_kp

    original_pointcloud.points = o3d.utility.Vector3dVector(np_pts)
    original_pointcloud.paint_uniform_color([0.2, 1, 0])
    o3d.visualization.draw_geometries([original_pointcloud, keypoints])

def keypoints_to_spheres(keypoints):
    spheres = o3d.geometry.TriangleMesh()
    for keypoint in keypoints.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
        sphere.translate(keypoint)
        spheres += sphere
    return spheres

def visualize_kp_in_pcd(key_pcd, original_pcd):
    original_pcd.paint_uniform_color([0.5, 0.5, 0.5])
    key_spheres = keypoints_to_spheres(key_pcd)
    key_spheres.paint_uniform_color([1, 0, 0])

    o3d.visualization.draw_geometries([original_pcd, key_spheres])

def visualize_kp_only(key_pcd):
    key_spheres = keypoints_to_spheres(key_pcd)
    key_spheres.paint_uniform_color([1, 0, 0])

    o3d.visualization.draw_geometries([key_spheres])
    
if __name__ == "__main__":
    np_pts = 1
#    np_pts = ReadPointCloudFromFile_ModelNet40("/home/han/Projects/Datasets/modelnet40_normal_resampled/airplane/airplane_0001.txt")
 #   choice = np.random.choice(len(np_pts[0]), 50, replace=True) # replace = True?
 #   np_key_pts = np_pts[0:3, choice]
#    Visualize(np_pts, np_key_pts)