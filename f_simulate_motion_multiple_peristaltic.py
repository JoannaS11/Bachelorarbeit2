import numpy as np
import open3d as o3d
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


def reverse_direction(b_spline):
    b_spline.reverse()


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


def combine_fcts_two_fct(t_on_line, zero_ones_array, *args):
    values = 0

    for arg in range(len(args[0])):
        m = args[0][arg]
        k = m(t_on_line)
        values += zero_ones_array * subtract_factor * k
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
            segments_array[i, 0] = (
                min_distances[i, 0]
                - (min_distances[i, 0] - min_distances[i - 1, 0]) * div_factor_lower
            )

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


def list_of_functions(segments_array, motion_fct, two_functions_for_one_segment=True):
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


def simulate_motion_parallel_2(
    b_Spline, pcd_colon, min_distances, vector_to_line, t_on_line, speed
):
    # two waves starting in the middle of spline

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
    list_fct = list_of_functions(
        segments_array, motion_fct, two_functions_for_one_segment
    )

    # precalculation of zero-ones arrays (1: segment with contraction, -1: relaxation, 0: nothing)
    locations_ones = np.zeros(
        (int(np.ceil(np.shape(min_distances)[0] / 2)), 2), dtype=int
    )
    locations_minus_ones = np.zeros(
        (int(np.ceil(np.shape(min_distances)[0] / 2)), 2), dtype=int
    )
    for min_dist in range(int(np.ceil(np.shape(min_distances)[0] / 2))):
        if np.shape(min_distances)[0] % 2 == 0:
            half_f = int(np.shape(min_distances)[0] / 2)
            min_dist_oral = half_f - 1 - int(min_dist % half_f)
            min_dist_oral_smaller = half_f - 1 - int((min_dist - 1) % half_f)
            min_dist_aboral = half_f + int(min_dist % half_f)
            min_dist_aboral_smaller = half_f + int((min_dist - 1) % half_f)
        else:
            half_f = int(np.ceil(np.shape(min_distances)[0] / 2))
            min_dist_oral = (half_f - 1) - int(min_dist % half_f)
            min_dist_oral_smaller = (half_f - 1) - int((min_dist - 1) % half_f)
            min_dist_aboral = half_f + int(min_dist % (half_f - 1))
            min_dist_aboral_smaller = half_f + int((min_dist - 1) % (half_f - 1))

        locations_ones[min_dist, 0] = int(min_dist_oral)
        locations_ones[min_dist, 1] = int(min_dist_aboral)
        locations_minus_ones[min_dist, 0] = int(min_dist_oral_smaller)
        locations_minus_ones[min_dist, 1] = int(min_dist_aboral_smaller)

    frequency = np.shape(min_distances)[0] * 20
    start = True
    for min_dist in range(frequency):
        zero_ones_array = np.zeros([np.shape(segments_array)[0]])
        index = int(min_dist % np.ceil(np.shape(min_distances)[0] / 2))
        min_dist_oral = locations_ones[index, 0]
        min_dist_aboral = locations_ones[index, 1]
        zero_ones_array[locations_ones[index, 0]] = 1

        if (index == np.shape(locations_minus_ones)[0] - 1 and np.shape(min_distances)[0] % 2 != 0):
            pass
        else:
            zero_ones_array[locations_ones[index, 1]] = 1

        t_smallest_distance_arg = np.argwhere(
            t_on_line == min_distances[min_dist_oral, 0]
        )

        # calculations for stopping condition of while loop
        original_point = np.asarray(
            b_Spline.evaluate_single(min_distances[min_dist_oral, 0])
        )
        half_point = (original_point + 0.5 * vector_to_line[t_smallest_distance_arg[0], :])[0]
        dist_half_contr_point = math.dist(original_point, half_point)

        # calculate factor alpha
        if start:
            factor = combine_fcts_two_fct(t_on_line, zero_ones_array, list_fct)
            start = False
        else:
            zero_ones_array[locations_minus_ones[index, 0]] = -1
            if index == 0 and np.shape(min_distances)[0] % 2 != 0:
                pass
            else:
                zero_ones_array[locations_minus_ones[index, 1]] = -1
            factor = combine_fcts_two_fct(t_on_line, zero_ones_array, list_fct)

        factor = np.reshape(factor, [np.shape(factor)[0], 1])

        # while loop
        while (math.dist(original_point, moving_part[t_smallest_distance_arg[0]][0])> dist_half_contr_point):
            moving_part += speed * factor * vector_to_line

            vis.poll_events()
            vis.update_geometry(pcd_colon)
            vis.update_renderer()

    vis.run()
    vis.destroy_window()
