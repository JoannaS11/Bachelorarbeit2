import numpy as np
import open3d as o3d
from time import sleep
import math


def sigmoid_function(x):
    # x [0,1]:
    # sigmoid: auf x Werte zw.0 und 1 skaliert, um 0.5 in x Ri verschoben
    y = 1 / (1 + np.exp(np.maximum(np.minimum(-10 * (x - 0.5), 700), -700)))
    return y


def adapted_sigmoid_function(x):
    y = 1 / (1 + np.power((1 - x) * 0.5 / (x * (1 - 0.5)), 2))
    return y


def sqrt_function(x):
    return np.sqrt(x)


def one_div_x_function(x):
    return 1 / (-x) - 1.1


def sin_function(x):
    return np.sin(x * 0.5 * 0.318)


def cos_function(x):
    return -0.5 * np.cos(x * 0.318) + 0.5


def constant_value(x):
    return 0.5


def convert_array_to_pcd(np_array, color):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_array)
    pcd.paint_uniform_color(color)

    return pcd


def fct_wrapper(fct, border_low, border_high, a=1, d=0, mirrored=False):
    # fct: input in [0,1] results in values [0,1]
    length = border_high - border_low
    b = 1 / length
    c = border_low
    if mirrored:
        return lambda t_on_line: np.where(
            (t_on_line <= border_low) | (t_on_line > border_high),
            0,
            a * fct(1 - b * (t_on_line - c)) + d,
        )
    else:
        return lambda t_on_line: np.where(
            (t_on_line <= border_low) | (t_on_line > border_high),
            0,
            a * fct(b * (t_on_line - c)) + d,
        )


def combine_fcts_two_fct(t_on_line, reverse, *args):
    values = 0
    if reverse:
        subtract_factor = -1
    else:
        subtract_factor = 1

    for arg in range(len(args[0])):
        m = args[0][arg]
        k = m(t_on_line)
        values += subtract_factor * k
        if arg % 2 != 0:
            subtract_factor *= -1

    return values


def get_segments_array(min_distances, div_factor_lower=0.5, div_factor_higher=0.5):
    segments_array = np.ndarray(
        [np.shape(min_distances)[0], 3]
    )  # [lower_border, t_min_distance, higher_border]
    for i in range(np.shape(min_distances)[0]):
        segments_array[i, 1] = min_distances[i, 0]
        if i == 0:
            segments_array[i, 0] = 0
        else:
            segments_array[i, 0] = (min_distances[i, 0] - (min_distances[i, 0] - min_distances[i - 1, 0]) * div_factor_lower)

        if i == np.shape(min_distances)[0] - 1:
            segments_array[i, 2] = 1
        else:
            segments_array[i, 2] = (
                min_distances[i, 0]
                + (min_distances[i + 1, 0] - min_distances[i, 0]) * div_factor_higher
            )

    return segments_array


def factor_fct(
    segments_array, motion_fct, t_on_line, two_functions_for_one_segment=True
):
    list_fcts = []
    if two_functions_for_one_segment:
        for i in range(np.shape(segments_array)[0]):
            mirrored = False
            for k in range(2):
                list_fcts.append(
                    fct_wrapper(
                        motion_fct,
                        segments_array[i, k],
                        segments_array[i, k + 1],
                        mirrored=mirrored,
                    )
                )
                mirrored = True

    # for loop, use function to find factor
    factor_reversed = combine_fcts_two_fct(t_on_line, True, list_fcts)
    factor_original = combine_fcts_two_fct(t_on_line, False, list_fcts)
    return factor_original, factor_reversed


def simulate_motion_parallel_2(
    b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line, speed
):
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
    factor_original, factor_reversed = factor_fct(
        segments_array, motion_fct, t_on_line, two_functions_for_one_segment
    )

    reverse = False
    frequency = np.shape(min_distances)[0] * 20
    for min_dist in range(frequency):
        # calculate variables for while condition:
        if min_dist == frequency - 1:
            t_smallest_distance_arg = np.argwhere(
                t_on_line == min_distances[(min_dist + 1) % 2, 0]
            )
        else:
            t_smallest_distance_arg = np.argwhere(
                t_on_line == min_distances[min_dist % 2, 0]
            )

        # calculations and definitions for stopping condition of while loop
        original_point = np.asarray(
            b_Spline.evaluate_single(min_distances[min_dist % 2, 0])
        )
        half_point = (original_point - 0.5 * vector_to_line[t_smallest_distance_arg[0], :])[0]
        dist_half_contr_point = math.dist(original_point, half_point)

        if min_dist == frequency - 1:
            stop_condition = (lambda original_points, moving_parts: math.dist(original_points, moving_parts) < 2 * dist_half_contr_point)
        else:
            stop_condition = (lambda original_points, moving_parts: math.dist(original_points, moving_parts) > dist_half_contr_point)

        # calculate factor
        if reverse:
            # if condition for first iteration (max(0,f))
            if min_dist == 0:
                factor = np.fmax(0, factor_reversed)
            elif min_dist == frequency - 1:
                factor = np.fmin(0, factor_reversed)
            else:
                factor = factor_reversed
        else:
            if min_dist == 0:
                factor = np.fmax(0, factor_original)
            elif min_dist == frequency - 1:
                factor = np.fmin(0, factor_original)
            else:
                factor = factor_original
        factor = np.reshape(factor, [np.shape(factor)[0], 1])

        # while loop
        while stop_condition(
            original_point, moving_part[t_smallest_distance_arg[0]][0]
        ):
            if (math.dist(original_point, moving_part[t_smallest_distance_arg[0]][0]) <= 1.1 * dist_half_contr_point):
                slow_down_factor = 10 * (math.dist(original_point, moving_part[t_smallest_distance_arg[0]][0]) / dist_half_contr_point - 1)
                moving_part += (
                    0.4 * (1 + 1.5 * slow_down_factor) * speed * factor * vector_to_line
                )
            elif (math.dist(original_point, moving_part[t_smallest_distance_arg[0]][0])>= 1.9 * dist_half_contr_point):
                slow_down_factor = 10 * (2 - math.dist(original_point, moving_part[t_smallest_distance_arg[0]][0]) / (dist_half_contr_point))
                moving_part += (
                    0.4 * (1 + 1.5 * slow_down_factor) * speed * factor * vector_to_line
                )
            else:
                moving_part += speed * factor * vector_to_line

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
