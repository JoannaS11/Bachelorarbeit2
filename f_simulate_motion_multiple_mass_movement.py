
import numpy as np
import open3d as o3d
import math
import copy

def sigmoid_function(x):
    # x [0,1]: 
    # sigmoid: auf x Werte zw.0 und 1 skaliert, um 0.5 in x Ri verschoben
    y = 1 / (1 + np.exp(np.maximum(np.minimum(-10 * (x - 0.5), 700), -700)))
    return y

def adapted_sigmoid_function(x):
    y = 1 / (1 + np.power((1-x) * 0.5 / (x * (1-0.5)), 2))
    return y

def sqrt_function(x):
    return np.sqrt(x)

def one_div_x_function(x):
    return 1 / (-x) - 1.1

def sin_function(x):
    return np.sin(x * 0.5 *0.318)

def cos_function(x):
    return -0.5 * np.cos(x *0.318) + 0.5

def constant_value(x):
    return 0.5

def convert_array_to_pcd(np_array, color):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_array)
        pcd.paint_uniform_color(color)

        return pcd

def fct_wrapper(fct, border_low, border_high, a=1, d=0, mirrored = False):
    # fct: input in [0,1] results in values [0,1]
    length = border_high - border_low
    b = 1 / length
    c = border_low
    if mirrored:
        return lambda t_on_line: np.where((t_on_line <= border_low) | (t_on_line > border_high), 0, a * fct(1 - b * (t_on_line - c)) + d)
    else:
        return lambda t_on_line: np.where((t_on_line <= border_low) | (t_on_line > border_high), 0, a * fct(b * (t_on_line - c)) + d)

def get_segments_array(min_distances, div_factor_lower = 0.5, div_factor_higher=0.5):
    segments_array = np.ndarray([np.shape(min_distances)[0], 3]) # [lower_border, t_min_distance, higher_border]
    for i in range(np.shape(min_distances)[0]):
        segments_array[i,1] = min_distances[i,0]
        if i ==  0:
            segments_array[i,0] = 0
        else: 
            segments_array[i,0] = min_distances[i,0] - (min_distances[i,0] - min_distances[i-1,0]) * div_factor_lower

        if i == np.shape(min_distances)[0]-1:
            segments_array[i,2] = 1
        else:
            segments_array[i,2] = min_distances[i,0] + (min_distances[i+1,0] - min_distances[i,0]) * div_factor_higher

    return segments_array

def factor_fct(segments_array, motion_fct, t_on_line, two_functions_for_one_segment = True):
    list_fcts = []
    if two_functions_for_one_segment:
        for i in range(np.shape(segments_array)[0]):
            mirrored = False
            for k in range(2):
                list_fcts.append(fct_wrapper(motion_fct, segments_array[i, k], segments_array[i, k+1], mirrored=mirrored))
                mirrored = True

    # for loop, use function to find factor
    factor_reversed = combine_fcts_two_fct(t_on_line, True, list_fcts)
    factor_original = combine_fcts_two_fct(t_on_line, False, list_fcts)
    return factor_original, factor_reversed

def list_of_functions(segments_array, motion_fct, two_functions_for_one_segment = True):
    list_fcts = []
    if two_functions_for_one_segment:
        for i in range(np.shape(segments_array)[0]):
            mirrored = False
            for k in range(2):
                list_fcts.append(fct_wrapper(motion_fct, segments_array[i, k], segments_array[i, k+1], mirrored=mirrored))
                mirrored = True
    return list_fcts

def combine_fcts_two_fct(t_on_line, zero_ones_array, *args):
    values = 0
    arg_not_0 = np.argwhere(zero_ones_array != 0)
    for arg in arg_not_0:
        arg = arg[0]
        fct_1 = args[0][arg * 2]
        fct_2 = args[0][arg * 2 + 1]
        k = fct_1(t_on_line) + fct_2(t_on_line)
        values += zero_ones_array[arg] * k
    
    return values

def simulate_motion_parallel_2(b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line, speed, reverse):
    # initialize window
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)

    # define function
    motion_fct = sigmoid_function
    two_functions_for_one_segment = True
    moving_part = np.asarray(pcd_colon.points)

    # get segments_array
    segments_array = get_segments_array(min_distances)
    # scale and translate functions for each segments
    list_fct = list_of_functions(segments_array, motion_fct, two_functions_for_one_segment)

    frequency = np.shape(min_distances)[0]*20
    start = True
    divisor = 4
    
    num_mass_m_rest = int(np.shape(min_distances)[0] % divisor)
    if num_mass_m_rest == 0:
        num_mass_m_rest = divisor
    for min_dist_ in range(frequency):
        min_dist = min_dist_ % np.shape(min_distances)[0]
        zero_ones_array = np.zeros([np.shape(segments_array)[0]])

        if reverse:
            # switch order of spline
            min_dist_save = min_dist
            min_dist = np.shape(min_distances)[0] - 1 - min_dist

            if min_dist_save % divisor == 0 and min_dist_save != 0:
                zero_ones_array[min_dist + 1: min_dist + 5] = -1
            elif not start and min_dist == np.shape(min_distances)[0] - 1:
                zero_ones_array[0 : num_mass_m_rest] = -1
            elif start:
                start = False
            zero_ones_array[min_dist] = 1

            # calculate variables for while condition:
            
            t_smallest_distance_arg = np.argwhere(t_on_line == min_distances[min_dist, 0]) 

            original_point = np.asarray(b_Spline.evaluate_single(min_distances[min_dist, 0]))
            half_point = (original_point + 0.5 * vector_to_line[t_smallest_distance_arg[0], :])[0]
            dist_half_contr_point = math.dist(original_point, half_point)

            # calculate factor alpha
            factor = combine_fcts_two_fct(t_on_line, zero_ones_array, list_fct)
            factor = np.reshape(factor, [np.shape(factor)[0], 1])       
        else:
            if min_dist % divisor == 0 and min_dist != 0:
                zero_ones_array[min_dist - 4 : min_dist] = -1
            elif not start and min_dist == 0:
                zero_ones_array[np.shape(min_distances)[0] - num_mass_m_rest : np.shape(min_distances)[0]+1] = -1
            elif start:
                start = False
            zero_ones_array[min_dist] = 1

            # calculate variables for while condition:
            
            t_smallest_distance_arg = np.argwhere(t_on_line == min_distances[min_dist, 0]) 

            original_point = np.asarray(b_Spline.evaluate_single(min_distances[min_dist, 0]))
            half_point = (original_point + 0.5 * vector_to_line[t_smallest_distance_arg[0], :])[0]
            dist_half_contr_point = math.dist(original_point, half_point)

            # calculate factor alpha
            factor = combine_fcts_two_fct(t_on_line, zero_ones_array, list_fct)
            factor = np.reshape(factor, [np.shape(factor)[0], 1])

        # while loop
        while math.dist(original_point, moving_part[t_smallest_distance_arg[0]][0]) > dist_half_contr_point:
            moving_part += speed * factor * vector_to_line

            vis.poll_events()
            vis.update_geometry(pcd_colon)
            vis.update_renderer()

    vis.run()
    vis.destroy_window()

