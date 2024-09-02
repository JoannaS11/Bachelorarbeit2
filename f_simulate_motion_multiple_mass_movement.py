
import numpy as np
import open3d as o3d
from time import sleep
import math
import copy 
import concurrent.futures
import threading
import open3d.visualization

def sigmoid_function(x):
    # x [0,1]: 
    # sigmoid: auf x Werte zw.0 und 1 skaliert, um 0.5 in x Ri verschoben
    y = 1 / (1 + np.exp(-10 * (x - 0.5)))#min(max(np.tanh(5*(x-0.5))[0], 0), 1)#
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

def combine_fcts_two_fct(t_on_line, zero_ones_array, *args):
    values = 0

    for arg in range(len(args[0])):
        m = args[0][arg]
        k = m(t_on_line)
        values += zero_ones_array * subtract_factor * k
        if arg % 2 != 0:
            subtract_factor *= -1
    
    return values



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

def simulate_motion_parallel(b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line):
    # initialize window
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)

    # define function
    motion_fct = sigmoid_function
    two_functions_for_one_segment = True
    moving_part = np.asarray(pcd_colon.points)
    original_points = copy.deepcopy(np.asarray(pcd_colon.points))

    # get segments_array
    segments_array = get_segments_array(min_distances)
    # scale and translate functions for each segments
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
        
    reverse = False
    for min_dist in range(np.shape(min_distances)[0]*6):
        # calculate variables for while condition:
        t_smallest_distance_arg = np.argwhere(t_on_line == min_distances[min_dist % 2, 0])
        contraction_point = np.asarray(b_Spline.evaluate_single(min_distances[min_dist % 2, 0]))
        half_point = (original_points[t_smallest_distance_arg[0], :] + 0.5 * vector_to_line[t_smallest_distance_arg[0], :])[0]
        dist_half_contr_point = math.dist(contraction_point, half_point)
        
        if reverse:
            # if condition for first iteration (max(0,f))
            if min_dist == 0:
                factor = np.fmax(0, factor_reversed)
            else:
                factor = factor_reversed
        else:
            if min_dist == 0:
                factor = np.fmax(0, factor_original)
            else:
                factor = factor_original
        factor = np.reshape(factor, [np.shape(factor)[0], 1])
        # while loop
        while math.dist(contraction_point, moving_part[t_smallest_distance_arg[0]][0]) > dist_half_contr_point:
            moving_part += 0.01 * factor * vector_to_line

            vis.poll_events()
            vis.update_geometry(pcd_colon)
            vis.update_renderer()
            sleep(0.01)
        # after while loop: f = -f
        if reverse:
            reverse = False
        else:
            reverse = True
    
    vis.run()
    vis.destroy_window()

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

def set_enabled(widget, enable):
    widget.enabled = enable
    for child in widget.get_children():
        child.enabled = enable

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

def simulate_motion_parallel_2(b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line):
    # initialize window
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_colon)
    speed = 0.01
    """speed_slider = open3d.visualization.gui.Slider(open3d.visualization.gui.Slider.DOUBLE)
    em = 1
    spacing = int(np.round(0.25 * em))
    fixed_prop_grid = open3d.visualization.gui.VGrid(2, spacing, open3d.visualization.gui.Margins(em, 0, em, 0))
    fixed_prop_grid.add_child(speed_slider)
    set_enabled(fixed_prop_grid, True)"""

    # define function
    motion_fct = sigmoid_function
    two_functions_for_one_segment = True
    moving_part = np.asarray(pcd_colon.points)
    original_points = copy.deepcopy(np.asarray(pcd_colon.points))

    # get segments_array
    segments_array = get_segments_array(min_distances)
    # scale and translate functions for each segments
    list_fct = list_of_functions(segments_array, motion_fct, two_functions_for_one_segment)

    #zero_ones_array = np.zeros([np.shape(segments_array)[0], 1])
        
    #reverse = False
    frequency = np.shape(min_distances)[0]*6
    x = 0
    start = True
    divisor = 4
    num_mass_m = int(np.shape(min_distances)[0] / divisor)
    num_mass_m_rest = int(np.shape(min_distances)[0] % divisor)
    if num_mass_m_rest == 0:
        num_mass_m_rest = divisor
    for min_dist_ in range(frequency):
        min_dist = min_dist_ % np.shape(min_distances)[0]
        zero_ones_array = np.zeros([np.shape(segments_array)[0]])

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

        factor = combine_fcts_two_fct(t_on_line, zero_ones_array, list_fct)

        factor = np.reshape(factor, [np.shape(factor)[0], 1])

        if min_dist == frequency - 1:
            stop_condition = lambda original_points, moving_parts: math.dist(original_points, moving_parts) < 2 * dist_half_contr_point
        else:
            stop_condition = lambda original_points, moving_parts: math.dist(original_points, moving_parts) > dist_half_contr_point
        # while loop
        while math.dist(original_point, moving_part[t_smallest_distance_arg[0]][0]) > dist_half_contr_point:
            moving_part += 0.04 * factor * vector_to_line

            vis.poll_events()
            vis.update_geometry(pcd_colon)
            vis.update_renderer()


    
    vis.run()
    vis.destroy_window()


