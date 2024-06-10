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
    points = np.asarray(pcd.points)
    t_old = 0.1
    #p = points[4000]
    #print(p[1])
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    half_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])
    for i in range(np.shape(points)[0]):
        # approsimate start value to be determined
        p = points[i]
        t_old = 0.1
        for k in range(30):
            #print(t_old)
            point_on_spline = bSpline.derivatives(t_old, 2)
            #print(point_old_2)
            f_t = (point_on_spline[0][0] - p[0]) * point_on_spline[1][0] + (point_on_spline[0][1] - p[1]) * point_on_spline[1][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[1][2] 
            f__t = point_on_spline[1][0] * point_on_spline[1][0] + point_on_spline[1][1] * point_on_spline[1][1] + point_on_spline[1][2] * point_on_spline[1][2] + (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[2][2]
            t = t_old - f_t / f__t       
            if t < 0.00:
                t_old = 0.0
                break
            elif t > 1.00:
                t_old = 1
                print(i)
                break
            #print(i)
            """if np.abs(t - t_old) < 0.000010:
                t_old = t
                break"""
 
            t_old = t                                                                       
        
        vector_to_line[i] = bSpline.evaluate_single(t_old) - p
        t_on_line[i] = t_old
        #print(vector_to_line)

        half_line[i] = p + 0.2 * vector_to_line[i]
        #print(half_line[i])

    x = np.argwhere((t_on_line < 0.02))
    x = np.argwhere(t_on_line[x[:]] > 0.0099)
    print(t_on_line)
    half_line[x[:]] = points[x[:]] + 0.7 * vector_to_line[x[:]]
    
    
    return half_line


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
    path_zyl_simple = os.path.join(os.getcwd(), "output_new", "2024-06-07_16-01-05-609625_0.4_min_path.ply")

    # read pointcloud and convert to array
    pcd = o3d.io.read_point_cloud(path_zyl_simple)

    
    line_bSpline = get_bSpline(pcd, sample_size)

    p = get_closest_point_on_spline(pcd_colon, line_bSpline)
    #p = line_bSpline.evaluate_single(get_closest_point_on_spline(pcd_colon, line_bSpline))
    """print(type(p))
    p_np = np.asarray([p, p])
    #p_np = 
    print(np.shape(p_np))
    print(p_np)
    x = np.array([[0.9, 0.7, 0.3],[1,2,3]])"""
    # create point cloud from curve
    print(p)
    p_pcd = o3d.geometry.PointCloud()    
    p_pcd.points = o3d.utility.Vector3dVector(p)#p_np)
    p_pcd.paint_uniform_color([1,0,1])

    # convert Spline to pcd and export
    line_pcd = convert_bSpline_to_pcd(line_bSpline)
    line_pcd.paint_uniform_color([1,0,0])
    export_pcd_as_ply(line_pcd, "output_curve", f"curve_as_pointcloud-{sample_size}")

    colors_line = np.asarray(pcd_colon.colors)
    colors_line[4000] = [1,0,0]
    pcd_colon.colors = o3d.utility.Vector3dVector(colors_line)
    # visualize
    o3d.visualization.draw_geometries([line_pcd, pcd_colon, p_pcd],
        mesh_show_wireframe = True,
        mesh_show_back_face = True,
        point_show_normal = True)

    #visualize(pcd_colon)


if __name__ == "__main__": main()  
