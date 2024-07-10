
import numpy as np
import open3d as o3d
from time import sleep
import math
import copy 

def sigmoid_function(x):
    # x [0,1]: 
    # sigmoid: auf x Werte zw.0 und 1 skaliert, um 0.5 in x Ri verschoben
    y = 1 / (1 + np.exp(-14 * (x - 0.5)))
    return y

#def linear_funcation(x)

def simulate_motion(b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line):
    # create window and add colon_data
    vis = o3d.visualization.Visualizer()
    vis.create_window()
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
            t_contr_oral_arg_old = np.argwhere((t_on_line > oral_half_old) & (t_on_line <= min_distances[min_dist-1, 0]))
            t_contr_aboral_arg_old = np.argwhere((t_on_line > min_distances[min_dist-1, 0]) & (t_on_line < aboral_half_old))
            
            # calculate linear fading factor for aboral and oral direction for previous min
            multi_factor_oral_old = 1 - (np.abs(min_distances[min_dist - 1, 0] - t_on_line[t_contr_oral_arg_old])) / np.abs(min_distances[min_dist-1, 0] - oral_half_old)
            multi_factor_aboral_old = 1 - (np.abs(min_distances[min_dist - 1, 0] - t_on_line[t_contr_aboral_arg_old])) / np.abs(min_distances[min_dist-1, 0] - aboral_half_old)
        
            multi_factor_oral_old = sigmoid_function(multi_factor_oral_old)
            multi_factor_aboral_old = sigmoid_function(multi_factor_aboral_old)

        # calculate mid_point between two mins on spline in aboral direction
        if min_dist == np.shape(min_distances)[0] - 1:
            # last min doesn't have a next min
            aboral_half = 1
        else:
            aboral_half = (min_distances[min_dist, 0] + min_distances[min_dist + 1, 0]) / 2

        # get all t included for this min
        t_contr_oral_arg = np.argwhere((t_on_line > oral_half) & (t_on_line <= min_distances[min_dist, 0]))
        t_contr_aboral_arg = np.argwhere((t_on_line > min_distances[min_dist, 0]) & (t_on_line < aboral_half))

        # calculate linear fading factor for aboral and oral direction for previous min
        multi_factor_oral = 1 - (np.abs(min_distances[min_dist, 0] - t_on_line[t_contr_oral_arg])) / np.abs(min_distances[min_dist, 0] - oral_half)
        multi_factor_aboral = 1 - (np.abs(min_distances[min_dist, 0] - t_on_line[t_contr_aboral_arg])) / np.abs(min_distances[min_dist, 0] - aboral_half)

        multi_factor_oral = sigmoid_function(multi_factor_oral)
        multi_factor_aboral = sigmoid_function(multi_factor_aboral)

        # contract at min point until shortest distance was contracted to half its length
        while math.dist(contraction_point, moving_part[t_smallest_distance_arg[0]][0]) > dist_half_contr_point:
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

    vis.run()
    vis.destroy_window()