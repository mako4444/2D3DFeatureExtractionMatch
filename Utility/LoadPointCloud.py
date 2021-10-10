import numpy as np
import open3d as o3d

def ReadPointCloudFromFile_ModelNet40(fileFullPath):
    np_pts = np.zeros(0)

    file = fileFullPath.split("/")[-1]

    print("Loading point cloud {} ...".format(file))

    with open(fileFullPath, 'r') as f:
        pts = []
        for line in f:
            one_pt = list(map(float, line[:-1].split(',')))
            pts.append(one_pt[:3])
        np_pts = np.array(pts)

    print("Loaded {} points from {}.".format(len(np_pts), file))
    # [n x 3] to 3 x n
    np_pts = np_pts.transpose()

    return np_pts

def read_from_pcd_file(file_full_path):
    pcd = o3d.io.read_point_cloud(file_full_path)
    return pcd


if __name__ == "__main__":
    ReadPointCloudFromFile_ModelNet40("/home/han/Projects/Datasets/modelnet40_normal_resampled/airplane/airplane_0001.txt")
    