import numpy as np
import open3d as o3d

def fct(x):
    return x + 5

def main():
    x = np.array([1,2,6,-2])
    k = np.zeros(np.shape(x))
    l = np.fmax(x, 0)
    m = np.where(x < 0,0,fct(x))
    #print(m)

    path_colon = "/home/yn86eniw/gaussian-splatting/output/colon_part_19_08/point_cloud/iteration_30000/point_cloud.ply"

    pcd_colon = o3d.io.read_point_cloud(path_colon)
    print(np.shape(pcd_colon.points))
    print(np.asarray(pcd_colon.points))

    o3d.visualization.draw_geometries(
        [pcd_colon],#, midline_pcd, min_distances_pcd],
        mesh_show_wireframe=True,
        mesh_show_back_face=True,
        point_show_normal=True,
    )

if __name__=="__main__": main()