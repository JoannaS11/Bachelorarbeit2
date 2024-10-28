
import os
import argparse
import open3d as o3d
import numpy as np
from typing import Union
#from open3d import geometry.PointCloud
#import open3d.geometry.PointCloud

#https://gist.github.com/AlexPasqua/f34994a0b0f8ee33f8730d6e660d55b5


def read_pcd(path: str) -> np.ndarray:
    """
    Reads a pointcloud with open3d and returns it as a numpy ndarray
    Args:
        path (str): path to the pointcloud to be read or the directory containing it/them
    Returns:
        np.ndarray: the pointcloud(s) as a numpy ndarray (dims: (pointcloud x) points x coordinates)
    """
    if os.path.isdir(path):
        pointclouds = []
        filenames = os.listdir(path)
        for filename in filenames:
            if filename[-4:] != '.pcd':
                continue
            pcd = o3d.io.read_point_cloud(path)
            pointclouds.append(np.asarray(pcd.points))
        return np.array(pointclouds)
    
    elif os.path.isfile(path):
        pcd = o3d.io.read_point_cloud(path)
        return np.asarray(pcd.points)


def pcd_to_bin(pcd: Union[str, o3d.open3d.geometry.PointCloud, np.ndarray], out_path: str):
    """
    Convert pointcloud from '.pcd' to '.bin' format
    Args:
        pcd (Union[str, PointCloud, np.ndarray]): the pointcloud to be converted (either its path, or the pointcloud itself)
        out_path (str): the path to the destination '.bin' file
    """
    # if 'pcd' is a string, assume it's the path to the pointcloud to be converted
    if isinstance(pcd, str):
        pcd = read_pcd(path=pcd)
    # save the poinctloud to '.bin' format
    out_path += "" if out_path[-4:] == ".bin" else ".bin"
    pcd.tofile(out_path)



path = "/home/yn86eniw/2d-gaussian-splatting/data/perfect_recons_virt_col_mapper_3/distorted/sparse/0/points3D.ply"
pcd = o3d.io.read_point_cloud(path)
pcd_to_bin("/home/yn86eniw/2d-gaussian-splatting/data/perfect_recons_virt_col_mapper_3/distorted/sparse/0/points3D.ply", "/home/yn86eniw/2d-gaussian-splatting/data/perfect_recons_virt_col_mapper_3/distorted/sparse/0/points3D.bin")
#pcd = o3d.io.read_point_cloud(path)
#
# o3d.io.write_point_cloud("points3D.txt", pcd)


