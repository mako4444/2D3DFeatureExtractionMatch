import numpy as np
import open3d as o3d

import os.path as osp
import sys
this_dir = osp.dirname(__file__) # the path of current file
project_root = osp.abspath(osp.join(this_dir, '../..')) #得到绝对路径
sys.path.append(project_root)
from utils.load_data_helper import ReadPointCloudFromFile_ModelNet40
from utils.visualizer import Visualize

class ISSDetector:
    # weight_radius - the considered range when calculating the weight of a neighbour point
    #                 smaller radius makes the key point closer to the corner
    # r_threshold - [0, 1]. the threshold of how good is the point. It's the threshold of lambda3, higher is better
    # nms_radius - the density of the final result, higher is sparser
    # gamma_21 - the ratio between eigenvalue 2 and 1, smaller means the corner is sharper
    # gamma_32 - the ratio between eigenvalue 2 and 1, smaller means the corner is sharper
    def __init__(self, np_pts, weight_radius, r_threshold, nms_radius, gamma_21=0.75, gamma_32=0.75):
        self._np_pts = np_pts # 3 x n
        self._weight_radius = weight_radius
        self._kdtree = o3d.geometry.KDTreeFlann(np_pts)
        self._r_threshold = r_threshold
        self._nms_radius = nms_radius
        self._gamma_21 = gamma_21
        self._gamma_32 = gamma_32

    def _get_neighbor(self, pt_index):
        # the key is to make sure the axis=0 is 3
        query_pt = np.array(self._np_pts[0:3, pt_index])
        [k, idx, _] = self._kdtree.search_radius_vector_3d(query_pt, radius=self._weight_radius)
        return k, idx

    def _get_weight(self, pt_index):
        k, _ = self._get_neighbor(pt_index)
        if (k <= 0):
            weight = 1
        else:
            weight = float(1) / float(k)
        
        return weight

    def _get_weighted_covmat(self, pt_i_index):
        k, pt_j_indices = self._get_neighbor(pt_i_index)

        numerator = np.zeros(9).reshape(3, 3)
        denominator = 0.
        covmat = numerator

        if (k > 0):
            for pt_j_index in pt_j_indices:
                w_j = self._get_weight(pt_j_index)
                p_i = np.array(self._np_pts[0:3, pt_i_index])
                p_j = np.array(self._np_pts[0:3, pt_j_index])
                temp = np.dot(np.expand_dims(p_j - p_i, axis=1), np.expand_dims(p_j - p_i, axis=0))
                numerator += w_j * temp
                denominator += w_j              
     
            covmat = numerator / denominator  
          
        return k, covmat

    def _get_eigenvalues(self, covmat):
        eigenvectors, eigenvalues, _ = np.linalg.svd(covmat, full_matrices=True)
        sort = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[sort]
        eigenvectors = eigenvectors[:, sort]
        # 1 x 3
        return eigenvalues

    # np_pts_eigenvalues - 3 x n
    def _eigenvalue_criterion_check(self, np_pts_eigenvalues):
        key_pts_indices = []

        # normalize eigenvalues according to the lambda 3
        max_lambda3_index = np.argmax(np_pts_eigenvalues[2])
        scale = np_pts_eigenvalues[2, max_lambda3_index]
        np_pts_eigenvalues = np_pts_eigenvalues / scale

        # lambda 3 threshold and criterion filtering, return key point index only 
        for pt_index, pt_eigenvalue_lambda3 in enumerate(np_pts_eigenvalues[2]):
            # filtering with the gamma 
            # filtering with the threshold
            pt_eigenvalue_lambda1 = np_pts_eigenvalues[0, pt_index]
            pt_eigenvalue_lambda2 = np_pts_eigenvalues[1, pt_index]

            if pt_eigenvalue_lambda2/pt_eigenvalue_lambda1 < self._gamma_21 and pt_eigenvalue_lambda3/pt_eigenvalue_lambda2 < self._gamma_32 and pt_eigenvalue_lambda3 > self._r_threshold:
                key_pts_indices.append(pt_index)
        
        return key_pts_indices

    def _nms(self, key_pts_indices, np_pts_eigenvalues):
        np_key_pts = self._np_pts[:, key_pts_indices]
        np_key_pts_eigenvalues = np_pts_eigenvalues[:, key_pts_indices] # 0 - keypointCount
        
        activated_key_pts_indeices = np.arange(len(np_key_pts[0])) # 0 - keypointCount

        output_key_pts = []

        while len(activated_key_pts_indeices) > 0:
            activated_key_pts = np_key_pts[:, activated_key_pts_indeices]
            activated_key_pts_eigenvalues = np_key_pts_eigenvalues[:, activated_key_pts_indeices]
            key_pts_kdtree = o3d.geometry.KDTreeFlann(activated_key_pts)
            # find the point with max lambda3
            max_index = np.argmax(activated_key_pts_eigenvalues[2])
            max_point = activated_key_pts[:, max_index]
    
            # find max point's neighbours
            [k, indics, _] = key_pts_kdtree.search_radius_vector_3d(max_point, radius=self._nms_radius)

            output_key_pts.append(max_point)

            # remove max point and its neighbours from the index array.
            for index in indics:
                if index in activated_key_pts_indeices:
                    activated_key_pts_indeices = np.delete(activated_key_pts_indeices, np.where(activated_key_pts_indeices==index))

        np_output_key_pts = np.array(output_key_pts)
        np_output_key_pts = np_output_key_pts.transpose()
        return np_output_key_pts

    def detect_ISS(self):
        pts_eigenvalues = []
        for pt_index,_ in enumerate(self._np_pts[0]):
            k, covmat = self._get_weighted_covmat(pt_index)
            if (k <= 0):
                continue
            pts_eigenvalues.append(self._get_eigenvalues(covmat))
        
        # n x 3
        np_pts_eigenvalues = np.array(pts_eigenvalues)
        # 3 x n
        np_pts_eigenvalues = np_pts_eigenvalues.transpose()
        key_pts_indices = self._eigenvalue_criterion_check(np_pts_eigenvalues)
        np_key_pts = self._nms(key_pts_indices, np_pts_eigenvalues)
        #np_key_pts = self._np_pts[:, key_pts_indices]
        # 3 x m
        return np_key_pts
    
if __name__ == "__main__":
    np_pts = ReadPointCloudFromFile_ModelNet40("/home/han/Projects/Datasets/modelnet40_normal_resampled/airplane/airplane_0001.txt")
    # weight_radius - smaller can make the key point closer to the corner
    # r_threshod - [0, 1]. the threshold of how good is the point 
    # nms_radius - the density of the final result, higher is sparser
    # gamma_21 - the ratio between eigenvalue 2 and 1, smaller means the corner is sharper
    # gamma_32 - the ratio between eigenvalue 2 and 1, smaller means the corner is sharper
    iss_detector = ISSDetector(np_pts, weight_radius=0.025, r_threshold=0.1, nms_radius=0.1, gamma_21=0.5, gamma_32=0.3)
    np_key_pts = iss_detector.detect_ISS()

    Visualize(np_pts, np_key_pts)
    