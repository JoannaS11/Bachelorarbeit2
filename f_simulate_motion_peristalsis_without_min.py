import numpy as np
import open3d as o3d
from time import sleep
import copy 

# peristalsis without regarding the locations of the circular muscles
def simulate_motion_parallel(pcd_colon, vector_to_line, t_on_line):
    # initialize window
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    points = copy.deepcopy(np.asarray(pcd_colon.points))
    z = 0
    # copy by reference
    half_line = np.asarray(pcd_colon.points)
    for i in range(500):
        # d = |1 - 7 * |z % 1 - t_on_line||⁴
        dist_to_z = np.power(np.abs(1 - 7 * np.abs((z % 1.0) - t_on_line[:])), 4)
        
        # d only in area around z (z +-0.08)
        dist_to_z = np.where(np.abs(z % 1.0 - t_on_line) < 0.08, dist_to_z, 0 )

        # p_new = p + 0.7 * d * v
        half_line[:, 0] = points[:, 0] + 0.7 * np.multiply(dist_to_z, vector_to_line[:, 0])
        half_line[:, 1] = points[:, 1] + 0.7 * np.multiply(dist_to_z, vector_to_line[:, 1])
        half_line[:, 2] = points[:, 2] + 0.7 * np.multiply(dist_to_z, vector_to_line[:, 2])

        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(0.2)
        z+=0.01

    vis.run()
    vis.destroy_window()