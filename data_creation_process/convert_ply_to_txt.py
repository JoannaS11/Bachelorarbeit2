
import os
import argparse
import open3d as o3d
import numpy as np
from typing import Union
#from open3d import geometry.PointCloud
#import open3d.geometry.PointCloud


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



path = "/home/yn86eniw/Documents/2020-VirtuellerDarm_Gastromapper_experiment/colmap8/points3D.ply"
pcd = o3d.io.read_point_cloud(path)
pcd_to_bin(pcd, "/home/yn86eniw/Documents/2020-VirtuellerDarm_Gastromapper_experiment/colmap8/points3D")
#pcd = o3d.io.read_point_cloud(path)
#
# o3d.io.write_point_cloud("points3D.txt", pcd)


