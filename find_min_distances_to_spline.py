import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.signal import argrelmin
from tqdm.autonotebook import tqdm


def find_min_distances(
    vector_to_line_distances,
    t_on_line,
    bSpline,
    pcd_colon,
    bin_size,
    length_spline,
    plot_on=True,
):

    t_vec_combined = np.c_[t_on_line, vector_to_line_distances]

    if plot_on:
        """
        plot direct distances from points to spline
        x-axis = t in range [0,1]
        y-axis = distances
        """
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.scatter(t_vec_combined[:, 0], t_vec_combined[:, 1], color="r")
        ax.set_xlabel("t")
        ax.set_ylabel("distances")
        plt.show()
    # if a t appears several times in t_on_line, just save the minimal distance of this t
    # get location of t's which exist several times
    u, c = np.unique(t_on_line, return_counts=True)
    multiple_arg = np.argwhere(c != 1)
    t_multiple_values = u[multiple_arg]
    # for each t which exists more than once
    for x in range(np.shape(t_multiple_values)[0]):
        vec_multi_arg = np.argwhere(t_vec_combined[:, 0] == t_multiple_values[x][0])
        # get min
        min = np.min(t_vec_combined[vec_multi_arg[:], 1])
        t_vec_combined = np.delete(t_vec_combined, vec_multi_arg, axis=0)
        t_vec_combined = np.insert(
            t_vec_combined, 0, np.array([t_multiple_values[x][0], min]), axis=0
        )

    # sort [t, vector_to_line] by t
    ind = np.argsort(t_vec_combined[:, 0])
    t_vec_combined = t_vec_combined[ind]

    # find local mins with subdivision in bins
    local_mins = create_bins_find_local_mins(
        t_vec_combined, bin_size, length_spline, plot_on
    )

    return local_mins  # [min_t_values, min_distance_values]


def create_bins_find_local_mins(t_vec_combined, bin_size, length_spline, plot_on=True):
    min_bin_arg = np.zeros([int(1 / bin_size)])

    # combine distances to bins where only min is saved
    for b_s in range(int(1 / bin_size)):
        bin_arg = np.argwhere(
            (
                (t_vec_combined[:, 0] >= bin_size * b_s)
                & (t_vec_combined[:, 0] < (b_s * bin_size + bin_size))
            )
        )
        if len(bin_arg) == 0:
            min_bin_arg[b_s] = -1
            continue

        min_bin = np.argmin(t_vec_combined[bin_arg[:], 1])
        min_t = (t_vec_combined[bin_arg[:], 0])[min_bin]
        min_bin_arg[b_s] = np.argwhere(t_vec_combined[:, 0] == min_t)[0]

    min_bin_arg_wrong = np.argwhere(min_bin_arg == [-1])
    min_bin_arg = np.delete(min_bin_arg, min_bin_arg_wrong, axis=0)
    min_bin_arg_int = min_bin_arg.astype(int)
    min_bins = t_vec_combined[min_bin_arg_int]
    min_distances_vector = t_vec_combined[min_bin_arg_int, 1]

    # find local mins from the bin mins(less distances than before)
    arg = argrelmin(min_distances_vector, order=2)
    min_bin_arg_int = min_bin_arg.astype(int)
    z = min_bins[arg[:]]  # [min_t_values, min_distance_values]

    if plot_on:
        """
        plot distances(higher values with same t removed) and min_distances
        x-axis = t in range [0,1]
        y-axis = distances
        """
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.scatter(
            t_vec_combined[min_bin_arg_int[:], 0],
            t_vec_combined[min_bin_arg_int[:], 1],
            color="r",
        )
        ax.scatter(min_bins[arg[:], 0], min_bins[arg[:], 1], color="m")
        ax.set_xlabel("t")
        ax.set_ylabel("distances")
        plt.show()

    return z  # [min_t_values, min_distance_values]


def get_closest_point_on_spline_4(
    pcd, bSpline, normals_to_inside, mean_distance_point_to_point, plot_on=True
):
    """This function chooses closest point on spline as the corresponding point, ignoring the normal direction"""
    # get data
    bSpline.evaluate()
    points = np.asarray(pcd.points)
    no_startPoints = 10 * len(bSpline.knotvector)

    # initialize arrays
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])

    t_start = np.linspace(0, 1, no_startPoints)
    # get start point on spline
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, no_startPoints)))

    # iterate over all points of the point cloud
    for i in tqdm(range(np.shape(points)[0]), desc="Find Dist to line: "):
        p = points[i]

        # calculate distances from all start points to current point
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])

        # sort distances from start points to current point
        max_dist_sorted = np.argsort(dist)

        # vector from point to closest point on spline
        t_old = t_start[max_dist_sorted[0]]
        vector = bSpline.evaluate_single(t_old) - p
        t_on_line[i] = t_old
        vector_to_line[i] = vector

    vector_to_line_distances = np.zeros(np.shape(vector_to_line)[0])
    vector_to_line_distances[:] = np.abs(np.linalg.norm(vector_to_line[:], axis=1))
    """
        vector_to_line: vector from pcd points to spline
        t_on_line: corresponding t values on spline [0,1]
        vector_to line_distances: lengths of vector_to_line vectors
    """
    return vector_to_line, t_on_line, vector_to_line_distances
