import numpy as np
import open3d as o3d
import open3d.visualization
import open3d.visualization.gui as gui
import json
import os
import geomdl.exchange
import random
from time import sleep
import math



class Application:
    def __init__(self, json_file_path):
        self.main_vis = None


        current_dir = os.getcwd()

        with open(json_file_path, "r+") as input_file:
            input_liste = json.load(input_file)

            # extract data from json file
            dir_json = os.path.join(*input_liste["dir"])
            data_path = os.path.join(current_dir, *input_liste["data"])
            medial_axis_bspline_path = os.path.join(
                current_dir, *input_liste["dir"], *input_liste["medial_axis_spline"]
            )
            data_name = input_liste["data"][-1]
            data_name = data_name.replace(".ply", "")
            t_on_line_path = input_liste["t_on_line"][1:]
            local_min_path = input_liste["min_distances_values"][1:]

            # read point clouds
            self.pcd_data = o3d.io.read_point_cloud(data_path)
            self.medial_axis_bspline = geomdl.exchange.import_json(medial_axis_bspline_path)[0]

            # read motion array file
            motion_arrays = np.load(os.path.join(current_dir, dir_json, *t_on_line_path))
            self.t_on_line = motion_arrays["t_on_line"]
            self.vector_to_line = motion_arrays["vector_to_line"]
            vector_to_line_distances = motion_arrays["vector_to_line_distances"]
            min_distances = np.load(os.path.join(current_dir, dir_json, *local_min_path))
            self.min_distances = min_distances["local_mins"]

        self.window = gui.Application.instance.create_window("Colon Movement")    
        self.scene = gui.SceneWidget()
        self.scene.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([1, 1, 1, 1])
        self.scene

        self.window.add_child(self.scene)
        self.run()

        

    def run(self):
        pass
        #bbox = o3d.geometry.AxisAlignedBoundingBox([-10, -10, -10],
        #                                           [10, 10, 10])
        #self.main_vis.setup_camera(60, bbox, [0,0,0])
        #sphere = o3d.geometry.TriangleMesh.create_sphere(0.5)
        #sphere.compute_vertex_normals()
        mat = o3d.visualization.rendering.MaterialRecord()
        mat.base_color = [
            random.random(),
            random.random(),
            random.random(), 1.0
        ]
        mat.shader = "defaultLit"
        self.scene.scene.add_geometry("name", self.pcd_data, mat)


        #app.add_window(self.main_vis)

def motion_simulation(b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line):
    for min_dist_ in range(np.shape(min_distances)[0]*6):
        min_dist = min_dist_% np.shape(min_distances)[0]

        t_smallest_distance_arg = np.argwhere(t_on_line == min_distances[min_dist, 0])
        contraction_point = np.asarray(b_Spline.evaluate_single(min_distances[min_dist, 0]))
        half_point = (points[t_smallest_distance_arg[0], :] + 0.7 * vector_to_line[t_smallest_distance_arg[0], :])[0]
        dist_half_contr_point = math.dist(contraction_point, half_point)

        # calculate mid_point between two mins on spline in oral direction
        if min_dist == 0:
            print("here")
            # first min doesn't have previous min
            oral_half = 0
        else:
            oral_half = (min_distances[min_dist, 0] + min_distances[min_dist - 1, 0]) / 2
            
            # calculate mid_point between two mins on spline in oral direction for previous min
            if min_dist == 1:
                oral_half_old = 0
            else:
                oral_half_old = (min_distances[min_dist-1, 0] + min_distances[min_dist - 2, 0]) / 2

            # calculate mid_point between two mins on spline in aboral direction for previous min
            aboral_half_old = (min_distances[min_dist-1, 0] + min_distances[min_dist, 0]) / 2

            # get all t included for previous min
            t_contr_oral_arg_old = np.argwhere((t_on_line >= oral_half_old) & (t_on_line <= min_distances[min_dist-1, 0]))
            t_contr_aboral_arg_old = np.argwhere((t_on_line > min_distances[min_dist-1, 0]) & (t_on_line <= aboral_half_old))
            
            # calculate linear fading factor for aboral and oral direction for previous min
            multi_factor_oral_old = 1 - (np.abs(min_distances[min_dist - 1, 0] - t_on_line[t_contr_oral_arg_old])) / np.abs(min_distances[min_dist-1, 0] - oral_half_old)
            multi_factor_aboral_old = 1 - (np.abs(min_distances[min_dist - 1, 0] - t_on_line[t_contr_aboral_arg_old])) / np.abs(min_distances[min_dist-1, 0] - aboral_half_old)
        
            multi_factor_oral_old = motion_function(multi_factor_oral_old)
            multi_factor_aboral_old = motion_function(multi_factor_aboral_old)

        # calculate mid_point between two mins on spline in aboral direction
        if min_dist == np.shape(min_distances)[0] - 1:
            # last min doesn't have a next min
            aboral_half = 1
        else:
            aboral_half = (min_distances[min_dist, 0] + min_distances[min_dist + 1, 0]) / 2

        # get all t included for this min
        t_contr_oral_arg = np.argwhere((t_on_line >= oral_half) & (t_on_line <= min_distances[min_dist, 0]))
        t_contr_aboral_arg = np.argwhere((t_on_line > min_distances[min_dist, 0]) & (t_on_line <= aboral_half))

        # calculate linear fading factor for aboral and oral direction for previous min
        multi_factor_oral = 1 - (np.abs(min_distances[min_dist, 0] - t_on_line[t_contr_oral_arg])) / np.abs(min_distances[min_dist, 0] - oral_half)
        multi_factor_aboral = 1 - (np.abs(min_distances[min_dist, 0] - t_on_line[t_contr_aboral_arg])) / np.abs(min_distances[min_dist, 0] - aboral_half)

        multi_factor_oral = motion_function(multi_factor_oral)
        multi_factor_aboral = motion_function(multi_factor_aboral)

        # contract at min point until shortest distance was contracted to half its length
        while math.dist(contraction_point, moving_part[t_smallest_distance_arg[0]][0]) > dist_half_contr_point:
            """vis.poll_events()
            view_control = vis.get_view_control()
            cam_params = view_control.convert_to_pinhole_camera_parameters()
            p = b_Spline.evaluate_single(0)
            print(cam_params)
            #cam_params.extrinsic[:3,3] = p
            print(vis.size())
        
            view_control.convert_from_pinhole_camera_parameters(cam_params)
            vis.update_renderer()"""
            #vis = setup_camera(vis, b_Spline)
            moving_part[t_contr_oral_arg[:,0]] += 0.01 * multi_factor_oral * vector_to_line[t_contr_oral_arg[:,0]]
            moving_part[t_contr_aboral_arg[:,0]] += 0.01 * multi_factor_aboral * vector_to_line[t_contr_aboral_arg[:,0]]
            
            # decontract last one at the same time
            if min_dist != 0:
                moving_part[t_contr_oral_arg_old[:,0]] -= 0.01 * multi_factor_oral_old* vector_to_line[t_contr_oral_arg_old[:,0]]
                moving_part[t_contr_aboral_arg_old[:,0]] -= 0.01 * multi_factor_aboral_old* vector_to_line[t_contr_aboral_arg_old[:,0]]

            vis.poll_events()
            vis.update_geometry(pcd_colon)
            vis.update_renderer()
            sleep(0.03)

        # after last min point decontract at last min point to get in start condition
        if min_dist == np.shape(min_distances)[0] - 1:
            orig_point = (points[t_smallest_distance_arg[0], :])[0]
            dist_orig_contr_point = math.dist(contraction_point, orig_point)

            while math.dist(contraction_point, moving_part[t_smallest_distance_arg[0]][0]) < dist_orig_contr_point:

                moving_part[t_contr_oral_arg[:,0]] -= 0.01 * multi_factor_oral * vector_to_line[t_contr_oral_arg[:,0]]
                moving_part[t_contr_aboral_arg[:,0]] -= 0.01 * multi_factor_aboral * vector_to_line[t_contr_aboral_arg[:,0]]
                
                vis.poll_events()
                vis.update_geometry(pcd_colon)
                vis.update_renderer()
                sleep(0.03)










def main():
    current_dir = os.getcwd()
    path_sub_16_10_32 = os.path.join(current_dir, "output_main", "Colon_subtriangles_2__16-07-2024_10-10-28", "Colon_subtriangles_2__16-07-2024_10-10-28_json.json")

    json_file_path = path_sub_16_10_32

    gui.Application.instance.initialize()
    Application(json_file_path)
    gui.Application.instance.run()



if __name__=="__main__": main()