import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.signal import argrelmin
from tqdm.autonotebook import tqdm

def find_min_distances(vector_to_line_distances, t_on_line, bSpline, pcd_colon, bin_size, length_spline, plot_on=True):   
    # if a t appears several times in t_on_line, just save the min distance for this t
    t_vec_combined = np.c_[t_on_line, vector_to_line_distances]

    if plot_on:
        """
            plot direct distances from points to spline
            x-axis = t in range [0,1]
            y-axis = distances
        """
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.scatter(t_vec_combined[:,0], t_vec_combined[:,1], color= 'r')
        ax.set_xlabel('t')
        ax.set_ylabel('distances')
        plt.show()

    u, c = np.unique(t_on_line, return_counts=True)
    multiple_arg = np.argwhere(c != 1)
    t_multiple_values = u[multiple_arg]
    for x in range(np.shape(t_multiple_values)[0]):
        vec_multi_arg = np.argwhere(t_vec_combined[:,0] == t_multiple_values[x][0])
        min = np.min(t_vec_combined[vec_multi_arg[:],1])
        t_vec_combined = np.delete(t_vec_combined, vec_multi_arg, axis=0)
        t_vec_combined = np.insert(t_vec_combined, 0, np.array([t_multiple_values[x][0], min]), axis=0)

    # sort by t
    ind = np.argsort(t_vec_combined[:,0] )
    t_vec_combined = t_vec_combined[ind]

    # find local mins if subdivision in bins
    local_mins = create_bins_find_local_mins(t_vec_combined, bin_size, length_spline, plot_on)

    return local_mins # [min_t_values, min_distance_values]

def create_bins_find_local_mins(t_vec_combined, bin_size, length_spline, plot_on=True):
    min_bin_arg = np.zeros([int(1/bin_size)])

    print(bin_size)
    # combine distances to bins where only min is saved
    for b_s in range(int(1/bin_size)):
        #print(f"b_WS {b_s}")
        bin_arg = np.argwhere(((t_vec_combined[:,0] >= bin_size * b_s) & (t_vec_combined[:,0] <= (b_s * bin_size+ bin_size))))
        if len(bin_arg) == 0:
            continue
        min_bin = np.argmin(t_vec_combined[bin_arg[:],1])
        min_t = (t_vec_combined[bin_arg[:], 0]) [min_bin]
        min_bin_arg[b_s] = np.argwhere(t_vec_combined[:, 0] == min_t)

    min_bin_arg_int = min_bin_arg.astype(int)
    min_bins = t_vec_combined[min_bin_arg_int]
    min_distances_vector = t_vec_combined[min_bin_arg_int,1]

    # find local mins from the bin graph
    arg = argrelmin(min_distances_vector,order=2)
    min_bin_arg_int = min_bin_arg.astype(int)
    z = min_bins[arg[:]] # [min_t_values, min_distance_values]
    #print(f"min_bin {min_bins}")

    if plot_on:
        """
            plot distances(higher values with same t removed) and min_distances
            x-axis = t in range [0,1]
            y-axis = distances
        """
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.scatter(t_vec_combined[min_bin_arg_int[:],0], t_vec_combined[min_bin_arg_int[:],1], color= 'r')
        ax.scatter(min_bins[arg[:],0], min_bins[arg[:],1], color='m')
        ax.set_xlabel('t')
        ax.set_ylabel('distances')
        plt.show()

    return z # [min_t_values, min_distance_values]

def get_closest_point_on_spline(pcd, bSpline, normals_to_inside, plot_on=True):
    # get data
    bSpline.evaluate()
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    # initialize arrays
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])

    # get start point on spline
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, 40)))

    for i in tqdm(range(np.shape(points)[0]), desc="Find Dist to line: "):
        # approximate start value to be determined
        p = points[i]

        # calculate distances from startpoint to current point
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])

        # sort those distances
        max_dist_sorted = np.argsort(dist) 
        #print(max_dist_sorted)
        # 
        #print("new point")
        limit = np.min([6, np.shape(start_points)[0]])
        length_vector = np.inf
        for s_p in range(0, limit):
            # use closest start_point
            
            t_old = np.linspace(0, 1, 40)[max_dist_sorted[s_p]]
            #print("new point")
            #print(f"start point {t_old}")
            for k in range(10):
                # Newton minimization method to find closest point to spline
                point_on_spline = bSpline.derivatives(t_old, 2)
                f_t = (point_on_spline[0][0] - p[0]) * point_on_spline[1][0] + (point_on_spline[0][1] - p[1]) * point_on_spline[1][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[1][2] 
                f__t = point_on_spline[1][0] * point_on_spline[1][0] + point_on_spline[1][1] * point_on_spline[1][1] + (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] +(point_on_spline[0][2]- p[2]) * point_on_spline[2][2] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[2][2]
                t = t_old - f_t / f__t    
                #print(f_t / f__t)
                # clamp between 0 and 1
                if t < 0.00:
                    t_old = 0.0
                elif t > 1.00:
                    t_old = 1.0
                else: 
                    t_old = t.round(decimals=6)  
                #print(t_old)

            # vector from point to closest point on spline
            vector = bSpline.evaluate_single(t_old) - p
            if length_vector > np.linalg.norm(vector):
                #print(f"vector: {vector} and told {t_old}")
                #print("save Point")
                length_vector = np.linalg.norm(vector)
                t_on_line[i] = t_old
                vector_to_line[i] = vector

            """if normals_to_inside:
                # check if normal and vector look in  approximately the same direction -> disabled
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) > - 0.7: # 45 ° in both directions
                    # save all values and go to next point
                    t_on_line[i] = t_old
                    vector_to_line[i] = vector
                    #if i % 200 == 0:
                        #print(f"{np.round(i / np.shape(points)[0], decimals=3) * 100} % done")
                    break
            else:
                # check if normal and vector look in  approximately the opposite direction -> disabled
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) < 0.7: # 45 ° in both directions
                    # save all values and go to next point
                    t_on_line[i] = t_old
                    vector_to_line[i] = vector
                    #if i % np.round(np.shape(points)[0] * 0.01) == 0:
                        #print(f"{np.round(i / np.shape(points)[0], decimals=3) * 100} % done")
                    break
"""
    vector_to_line_distances = np.zeros(np.shape(vector_to_line)[0])
    vector_to_line_distances[:] = np.abs(np.linalg.norm(vector_to_line[:], axis=1))
    x = np.argwhere(vector_to_line == 0)
    #print(f"niczht x { vector_to_line_distances}")

    """
        vector_to_line: vector from pcd points to spline
        t_on_line: corresponding t values on spline [0,1]
        vector_to line_distances: lengths of vector_to_line vectors
    """
    return vector_to_line, t_on_line, vector_to_line_distances