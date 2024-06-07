import geomdl.fitting
import numpy as np
import geomdl
import os
import open3d as o3d
import scipy
import matplotlib as plt
import scipy.interpolate
from datetime import datetime
from time import sleep

def get_bSpline(pcd, sample_size):
    pcd_np = np.asarray(pcd.points)

    # interpolate line and upsample
    line_pc_array = np.ndarray.tolist(pcd_np)

    # x = geomdl.fitting.approximate_curve(line_pc_array, 2)
    b_spline = geomdl.fitting.interpolate_curve(line_pc_array, 2)
    b_spline.sample_size = sample_size * b_spline.sample_size

    return b_spline


def export_pcd_as_ply(pcd, output_folder, output_name):
    # get current date and time
    date_time = str(datetime.now())
    date_time = date_time.replace(".", "-").replace(":", "-")
    date_time = date_time.replace(" ", "_")
    # export as ply
    o3d.io.write_point_cloud(os.path.join(os.getcwd(), output_folder, f"{date_time}_{output_name}.ply"), pcd)


def convert_bSpline_to_pcd(bSpline):
    # get points from bSpline
    points_bSpline = bSpline.evalpts
    points_bSpline_np = np.array(points_bSpline)

    # create point cloud from curve
    line_pcd = o3d.geometry.PointCloud()    
    line_pcd.points = o3d.utility.Vector3dVector(points_bSpline_np)

    return line_pcd


def visualize(pcd_colon):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    for i in range(100):
        pcd_colon.paint_uniform_color(change_color()[0])
        vis.poll_events()
        vis.update_geometry(pcd_colon)
        vis.update_renderer()
        sleep(1)

    vis.register_animation_callback(move_colon)
    vis.run()
    vis.destroy_window()


def change_color():
    color = np.random.rand(1,3)
    print(color)
    
    return color
    #vis.get_view_Control()


def move_colon(vis):
    color = np.random.rand([3,1])

    #vis.get_view_Control()

def get_closest_point_on_spline(pcd, bSpline):
    bSpline.evaluate()
    print(f"splijne {len(bSpline)} and {bSpline.evaluate_single(0.501)}")
    return bSpline.evaluate_single(0.9999)


def find_smallest_dis_to_point(point, bspline):
    pass

def main():
    # adjustable parameter
    sample_size = 2

    # prepare colon for visualization
    path_z_simple = os.path.join(os.getcwd(), "data","zylinder_simple.ply")
    path_colon = os.path.join(os.getcwd(), "data/Colon.ply")
    pcd_colon = o3d.io.read_point_cloud(path_z_simple)
    pcd_colon.paint_uniform_color([1,1,0])
    
    # data paths
    path_ = os.path.join(os.getcwd(), "output_new","2024-05-27_08-22-28-505194_line_2.ply")
    path_1 = os.path.join(os.getcwd(), "output_new", "2024-05-29_09-31-42-719176_0.25_min_path.ply")
    path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-04_11-14-53-584648_0.4_min_path.ply")

    # read pointcloud and convert to array
    pcd = o3d.io.read_point_cloud(path_zyl_simple)

    line_bSpline = get_bSpline(pcd, sample_size)
    p = get_closest_point_on_spline(pcd_colon, line_bSpline)
    print(type(p))
    p_np = np.asarray([p, p])
    #p_np = 
    print(np.shape(p_np))
    print(p_np)
    x = np.array([[0.9, 0.7, 0.3],[1,2,3]])
    # create point cloud from curve
    p_pcd = o3d.geometry.PointCloud()    
    p_pcd.points = o3d.utility.Vector3dVector(p_np)
    p_pcd.paint_uniform_color([1,0,0])

    # convert Spline to pcd and export
    line_pcd = convert_bSpline_to_pcd(line_bSpline)
    line_pcd.paint_uniform_color([0,0,1])
    export_pcd_as_ply(line_pcd, "output_curve", f"curve_as_pointcloud-{sample_size}")


    # visualize
    o3d.visualization.draw_geometries([line_pcd, pcd_colon, p_pcd],

       mesh_show_wireframe = True,

    mesh_show_back_face = True,
    point_show_normal = True)

    #visualize(pcd_colon)


if __name__ == "__main__": main()  

"""
bspl = scipy.interpolate.make_interp_spline(pcd_np[:][0], pcd_np[:][1:3], k=5, axis=1)
xx = np.linspace(0, 2*np.pi, 100)
ax = plt.axes(projection='3d')
ax.plot3D(xx, *bspl(xx))
#ax.scatter3D(x, *y, color='red')
plt.show()"""
