
import numpy as np
import open3d as o3d
from time import sleep
import math
import copy 
import concurrent.futures
import threading

def sigmoid_function(x):
    # x [0,1]: 
    # sigmoid: auf x Werte zw.0 und 1 skaliert, um 0.5 in x Ri verschoben
    y = 1 / (1 + np.exp(-14 * (x - 0.5)))#min(max(np.tanh(5*(x-0.5))[0], 0), 1)#
    return y

def adapted_sigmoid_function(x):
    y = 1 / (1 + np.power((1-x) * 0.5 / (x * (1-0.5)), 2))
    return y

def constant_value(x):
    return 0.5

def reverse_direction(b_spline):
    b_spline.reverse()

def setup_camera(vis, b_spline):
    view_control = vis.get_view_control()
    cam_params = view_control.convert_to_pinhole_camera_parameters()
    p = b_spline.evaluate_single(0)
    cam_params.extrinsic = np.array([
        [1,0,0,p[0]],
        [0,1,0,p[1]],
        [0,0,1,p[2]],
        [0,0,0,1]
    ])
    view_control.convert_from_pinhole_camera_parameters(cam_params)
    vis.poll_events()
    vis.update_renderer()
    return vis
    #cam.set_position(b_spline.evaluate_single(0))
    #cam.look_at(b_spline.evaluate_single(0), b_spline.evaluate(0.001), [1,0,0])
    """def rotate_view(vis):

        ctr = vis.get_view_control()

        ctr.translate

        return False"""


    """o3d.visualization.draw_geometries_with_animation_callback([pcd],

                                                              rotate_view)"""
    #return cam

#def linear_funcation(x)

def simulate_motion(b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line):
    # create window and add colon_data
    
    motion_function = sigmoid_function#constant_value


    vis = o3d.visualization.Visualizer()
    vis.create_window()
    #setup_camera()
    vis.add_geometry(pcd_colon)
    # copy by value
    points = copy.deepcopy(np.asarray(pcd_colon.points))
    # copy by reference
    moving_part = np.asarray(pcd_colon.points)

    # iterate over min distances
    for min_dist_ in range(np.shape(min_distances)[0]*6):
        min_dist = min_dist_% np.shape(min_distances)[0]

        t_smallest_distance_arg = np.argwhere(t_on_line == min_distances[min_dist, 0])
        contraction_point = np.asarray(b_Spline.evaluate_single(min_distances[min_dist, 0]))
        half_point = (points[t_smallest_distance_arg[0], :] + 0.7 * vector_to_line[t_smallest_distance_arg[0], :])[0]
        dist_half_contr_point = math.dist(contraction_point, half_point)

        # calculate mid_point between two mins on spline in oral direction
        if min_dist == 0:
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
            vis.poll_events()
            view_control = vis.get_view_control()
            cam_params = view_control.convert_to_pinhole_camera_parameters()
            p = np.asarray(b_Spline.evaluate_single(0))
            #print(cam_params.extrinsic)
            """cam_params.extrinsic= np.array([[1,0,0,p[0]],
                                   [0,1,0,p[1]],
                                   [0,0,1,p[2]],
                                   [0,0,0,1]]) * cam_params.extrinsic"""
            #print(vis.size())
        
            view_control.convert_from_pinhole_camera_parameters(cam_params, allow_arbitrary=True)
            vis.update_renderer()
            #vis = setup_camera(vis, b_Spline)
            moving_part[t_contr_oral_arg[:,0]] += 0.01 * multi_factor_oral * vector_to_line[t_contr_oral_arg[:,0]]
            moving_part[t_contr_aboral_arg[:,0]] += 0.01 * multi_factor_aboral * vector_to_line[t_contr_aboral_arg[:,0]]
            
            # decontract last one at the same time
            if min_dist != 0:
                moving_part[t_contr_oral_arg_old[:,0]] -= 0.01 * multi_factor_oral_old * vector_to_line[t_contr_oral_arg_old[:,0]]
                moving_part[t_contr_aboral_arg_old[:,0]] -= 0.01 * multi_factor_aboral_old * vector_to_line[t_contr_aboral_arg_old[:,0]]

            vis.poll_events()
            vis.update_geometry(pcd_colon)
            vis.update_renderer()
            sleep(0.01)

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
                sleep(0.01)

    vis.run()
    vis.destroy_window()