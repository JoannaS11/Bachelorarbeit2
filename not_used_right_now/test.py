import numpy as np
import open3d as o3d
import torch

def fct(x):
    return x + 5


def main():
    
    a = torch.randn(1, 2, 3, 4, 5)
    empty_tensor =torch.FloatTensor()
    print(empty_tensor.shape)
    torch.numel(a)
    x = np.array([1,2,6,-2])
    k = np.zeros(np.shape(x))
    l = np.fmax(x, 0)
    m = np.where(x < 0,0,fct(x))

    path_bin = "/home/yn86eniw/2d-gaussian-splatting/data/virtual_col_mapper_render0_part0_317_28_8/sparse/0/cameras.bin"
    #print(m)

    path_colon = "/home/yn86eniw/gaussian-splatting/output/colon_part_19_08/point_cloud/iteration_30000/point_cloud.ply"

    #pcd_colon = o3d.io.read_point_cloud(path_colon)
    pcd = o3d.io.read_point_cloud("/home/yn86eniw/Documents/2020-VirtuellerDarm_Gastromapper_experiment/colmap_test2/points3D.txt", format="xyzrgb")
    #print(np.shape(pcd_colon.points))
    #print(np.asarray(pcd_colon.points))
    x = np.asarray(pcd.points)

    o3d.visualization.draw_geometries(
        [pcd],#_colon],#, midline_pcd, min_distances_pcd],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
        point_show_normal=True,
    )

if __name__=="__main__": main()