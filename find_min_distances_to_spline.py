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
        bin_arg = np.argwhere(((t_vec_combined[:,0] >= bin_size * b_s) & (t_vec_combined[:,0] < (b_s * bin_size+ bin_size))))
        if len(bin_arg) == 0:
            continue
        min_bin = np.argmin(t_vec_combined[bin_arg[:],1])
        min_t = (t_vec_combined[bin_arg[:], 0]) [min_bin]
        min_bin_arg[b_s] = np.argwhere(t_vec_combined[:, 0] == min_t)[0]

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

def get_closest_point_on_spline(pcd, bSpline, normals_to_inside, plot_on=True): #newton does not work
    no_startPoints = 40
    # get data
    bSpline.evaluate()
    print(len(bSpline.knotvector))
    points = np.asarray(pcd.points)

    # initialize arrays
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])


    # get start point on spline
    t_start = np.linspace(0, 1, no_startPoints)
    start_points = np.asarray(bSpline.evaluate_list(t_start))

    for i in tqdm(range(np.shape(points)[0]), desc="Find Dist to line: "):
        # approximate start value to be determined
        p = points[i]

        # calculate distances from startpoint to current point
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])

        # sort those distances
        #print("new point")
        #print(dist)
        max_dist_sorted = np.argsort(dist) 
        #print("sort")
        #print(max_dist_sorted)

        limit = np.min([6, np.shape(start_points)[0]])
        length_vector_min = np.inf
        for s_p in range(1):
            #print(max_dist_sorted[s_p])
            # use closest start_point
            t_old = t_start[max_dist_sorted[s_p]]
            if i > 8000 and i < 8050:
                print(f"new point {i}")
                print(f"t_start {t_old}")
            #print("new")
            for k in range(8):
                # Newton minimization method to find closest point to spline
                point_on_spline = bSpline.derivatives(t_old, 2)
                if i > 8000 and i < 8050:
                    print(f"derivatives {point_on_spline}")
                f_t = (point_on_spline[0][0] - p[0]) * point_on_spline[1][0] + (point_on_spline[0][1] - p[1]) * point_on_spline[1][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[1][2] 
                f__t_1 = point_on_spline[1][0] * point_on_spline[1][0] + point_on_spline[1][1] * point_on_spline[1][1] + point_on_spline[1][2] * point_on_spline[1][2]
                f__t_2 = (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[2][2]
                f__t = f__t_1 + f__t_2
                t = t_old - f_t / f__t  
                #t = t_old
                # clamp between 0 and 1
                if t < 0.00000000000000000001:
                    t_old = 0.0
                elif t > 0.999999999999999999:
                    t_old = 1.0
                else: 
                    t_old = t
                
                if i > 8000  and i < 8050:
                    print(f"t_intermediate {t_old}")

            if i > 8000  and i < 8050:
                print(f"t_end {t_old}")
            # vector from point to closest point on spline
            vector = bSpline.evaluate_single(t_old) - p
            #print(length_vector_min)
            if np.linalg.norm(vector) < length_vector_min:
                length_vector_min = np.linalg.norm(vector)
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

def get_closest_point_on_spline_2(pcd, bSpline, normals_to_inside, mean_distance_point_to_point, plot_on=True): #newton does not work
    # get data
    no_startPoints = 60
    bSpline.evaluate()
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    # initialize arrays
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])

    # get start point on spline
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, no_startPoints)))

    for i in tqdm(range(np.shape(points)[0]), desc="Find Dist to line: "):
        x=0
        t_set = False
        # approximate start value to be determined
        p = points[i]

        # calculate distances from startpoint to current point
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])

        # sort those distances
        max_dist_sorted = np.argsort(dist) 

        limit = np.min([8, np.shape(start_points)[0]])
        length_vector_min = np.inf
        for s_p in range(limit):
            # use closest start_point
            t_old = np.linspace(0, 1, no_startPoints)[max_dist_sorted[s_p]]

            for k in range(8):
                # Newton minimization method to find closest point to spline
                point_on_spline = bSpline.derivatives(t_old, 2)
                f_t = (point_on_spline[0][0] - p[0]) * point_on_spline[1][0] + (point_on_spline[0][1] - p[1]) * point_on_spline[1][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[1][2] 
                f__t_1 = point_on_spline[1][0] * point_on_spline[1][0] + point_on_spline[1][1] * point_on_spline[1][1] + point_on_spline[1][2] * point_on_spline[1][2]
                f__t_2 = (point_on_spline[0][0]- p[0]) * point_on_spline[2][0] + (point_on_spline[0][1]- p[1]) * point_on_spline[2][1] + (point_on_spline[0][2]- p[2]) * point_on_spline[2][2]
                f__t = f__t_1 + f__t_2
                t = t_old - f_t / f__t    

                # clamp between 0 and 1
                if t < 0.00:
                    t_old = 0.0
                elif t > 1.00:
                    t_old = 1.0
                else: 
                    t_old = t#.round(decimals=6)  

            # vector from point to closest point on spline
            vector = bSpline.evaluate_single(t_old) - p

            if normals_to_inside:
                # check if normal and vector look in  approximately the same direction -> disabled
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) > -0 and np.linalg.norm(vector) < 2 * mean_distance_point_to_point: # 45 ° in both directions
                    # save all values and go to next point
                    if length_vector_min > np.linalg.norm(vector):
                        length_vector_min = np.linalg.norm(vector)
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector
                        t_set = True

            else:
                # check if normal and vector look in  approximately the opposite direction -> disabled
                if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) < 0 and np.linalg.norm(vector) < 2 *mean_distance_point_to_point: # 45 ° in both directions
                    # save all values and go to next point
                    if length_vector_min > np.linalg.norm(vector):
                        length_vector_min = np.linalg.norm(vector)
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector
                        t_set = True

        if not t_set:

            for l in range(limit):
                t = np.linspace(0, 1, no_startPoints)[max_dist_sorted[l]]
                vector = bSpline.evaluate_single(t) - p
                if normals_to_inside:
                # check if normal and vector look in  approximately the same direction -> disabled
                    if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) > - 0 and np.linalg.norm(vector) < 2 * mean_distance_point_to_point: # 45 ° in both directions
                        print(f"here + {x}+  t {t}")
                    # save all values and go to next point
                        length_vector_min = np.linalg.norm(vector)
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector
                        x+=1
                        t_set = True
                        break

                else:
                    # check if normal and vector look in  approximately the opposite direction -> disabled
                    if np.dot(vector / np.linalg.norm(vector), normals[i] / np.linalg.norm(normals[i])) < 0 and np.linalg.norm(vector) < 2 * mean_distance_point_to_point: # 45 ° in both directions
                        # save all values and go to next point
                        print(f"here + {x} +  t {t}")
                        length_vector_min = np.linalg.norm(vector)
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector
                        t_set = True
                        x+=1
                        break
            if not t_set: 
                print("there")
                t = np.linspace(0, 1, no_startPoints)[max_dist_sorted[0]]
                vector = bSpline.evaluate_single(t) - p
                length_vector_min = np.linalg.norm(vector)
                t_on_line[i] = t_old
                vector_to_line[i] = vector
                t_set = True

    
    vector_to_line_distances = np.zeros(np.shape(vector_to_line)[0])
    vector_to_line_distances[:] = np.abs(np.linalg.norm(vector_to_line[:], axis=1))
    mean = np.mean(vector_to_line_distances)
    """
        vector_to_line: vector from pcd points to spline
        t_on_line: corresponding t values on spline [0,1]
        vector_to line_distances: lengths of vector_to_line vectors
    """
    return vector_to_line, t_on_line, vector_to_line_distances

def get_closest_point_on_spline_3(pcd, bSpline, normals_to_inside, mean_distance_point_to_point, plot_on=True): #choose closest point which is in angle, otherwise increase angle and try again
    # get data
    bSpline.evaluate()
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    no_startPoints = 10 * len(bSpline.knotvector)
    # initialize arrays
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])

    t_start = np.linspace(0, 1, no_startPoints)
    # get start point on spline
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, no_startPoints)))

    for i in tqdm(range(np.shape(points)[0]), desc="Find Dist to line: "):
        # approximate start value to be determined
        p = points[i]
        length_vector_min = np.inf

        # calculate distances from startpoint to current point
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])
         
        max_dist_sorted = np.argsort(dist) 

        limit = np.min([int(len(bSpline.knotvector) * 0.5), np.shape(start_points)[0]])
        length_vector_min = np.inf
        for s_p in range(limit):
        # vector from point to closest point on spline
            t_old = t_start[max_dist_sorted[s_p]]
            vector = bSpline.evaluate_single(t_old) - p
            vector_length = np.linalg.norm(vector)
            if normals_to_inside:
                # check if normal and vector look in  approximately the same direction -> disabled
                if np.dot(vector / vector_length, normals[i] / np.linalg.norm(normals[i])) > -0 and vector_length < 1 * mean_distance_point_to_point: # 45 ° in both directions
                    # save all values and go to next point
                    if length_vector_min > vector_length:
                        length_vector_min = vector_length
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector

            else:
                # check if normal and vector look in  approximately the opposite direction -> disabled
                if np.dot(vector / vector_length, normals[i] / vector_length) < 0 and vector_length < 1 * mean_distance_point_to_point: # 45 ° in both directions
                    # save all values and go to next point
                    if length_vector_min > vector_length:
                        length_vector_min = vector_length
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector
        angle = 0
        distance_multiplier = 1 
        while sum(vector_to_line[i]) == 0:
            angle += np.min([1, 0.1])
            for s_p in range(limit):
        # vector from point to closest point on spline
                t_old = t_start[max_dist_sorted[s_p]]
                vector = bSpline.evaluate_single(t_old) - p
                vector_length = np.linalg.norm(vector)
                if normals_to_inside:
                    # check if normal and vector look in  approximately the same direction -> disabled
                    if np.dot(vector / vector_length, normals[i] / np.linalg.norm(normals[i])) > -angle and vector_length < 1.5 * mean_distance_point_to_point: # 45 ° in both directions
                        # save all values and go to next point
                        if length_vector_min > vector_length:
                            length_vector_min = vector_length
                            t_on_line[i] = t_old
                            vector_to_line[i] = vector


                else:
                    # check if normal and vector look in  approximately the opposite direction -> disabled
                    if np.dot(vector / vector_length, normals[i] / np.linalg.norm(normals[i])) < angle and vector_length < 1.5 * mean_distance_point_to_point: # 45 ° in both directions
                        # save all values and go to next point
                        if length_vector_min > vector_length:
                            length_vector_min = vector_length
                            t_on_line[i] = t_old
                            vector_to_line[i] = vector

                #if angle == 1 and sum(vector_to_line[i]) == 0:
                    #angle = 0
                    #distance_mutliplier += 0.2
    
    vector_to_line_distances = np.zeros(np.shape(vector_to_line)[0])
    vector_to_line_distances[:] = np.abs(np.linalg.norm(vector_to_line[:], axis=1))
    mean = np.mean(vector_to_line_distances)
    """
        vector_to_line: vector from pcd points to spline
        t_on_line: corresponding t values on spline [0,1]
        vector_to line_distances: lengths of vector_to_line vectors
    """
    return vector_to_line, t_on_line, vector_to_line_distances


def get_closest_point_on_spline_4(pcd, bSpline, normals_to_inside, mean_distance_point_to_point, plot_on=True): # choose closest point on spline
    # get data
    bSpline.evaluate()
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    no_startPoints = 10 * len(bSpline.knotvector)
    # initialize arrays
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])

    t_start = np.linspace(0, 1, no_startPoints)
    # get start point on spline
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, no_startPoints)))

    for i in tqdm(range(np.shape(points)[0]), desc="Find Dist to line: "):
        # approximate start value to be determined
        p = points[i]
        length_vector_min = np.inf

        # calculate distances from startpoint to current point
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])
         
        max_dist_sorted = np.argsort(dist) 

        limit = np.min([len(bSpline.knotvector), np.shape(start_points)[0]])
        length_vector_min = np.inf
        for s_p in range(limit):
        # vector from point to closest point on spline
            t_old = t_start[max_dist_sorted[s_p]]
            vector = bSpline.evaluate_single(t_old) - p
            vector_length = np.linalg.norm(vector)
            if length_vector_min > vector_length:
                        length_vector_min = vector_length
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector
            
    vector_to_line_distances = np.zeros(np.shape(vector_to_line)[0])
    vector_to_line_distances[:] = np.abs(np.linalg.norm(vector_to_line[:], axis=1))
    mean = np.mean(vector_to_line_distances)
    """
        vector_to_line: vector from pcd points to spline
        t_on_line: corresponding t values on spline [0,1]
        vector_to line_distances: lengths of vector_to_line vectors
    """
    return vector_to_line, t_on_line, vector_to_line_distances

def get_closest_point_on_spline_5(pcd, bSpline, normals_to_inside, mean_distance_point_to_point, plot_on=True):
    # get data
    bSpline.evaluate()
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    no_startPoints = 10 * len(bSpline.knotvector)
    # initialize arrays
    vector_to_line = np.zeros([np.shape(points)[0], 3])
    t_on_line = np.zeros([np.shape(points)[0]])

    t_start = np.linspace(0, 1, no_startPoints)
    # get start point on spline
    start_points = np.asarray(bSpline.evaluate_list(np.linspace(0, 1, no_startPoints)))

    for i in tqdm(range(np.shape(points)[0]), desc="Find Dist to line: "):
        # approximate start value to be determined
        p = points[i]
        length_vector_min = np.inf

        # calculate distances from startpoint to current point
        dist = np.zeros(np.shape(start_points)[0])
        for po in range(np.shape(start_points)[0]):
            dist[po] = math.dist(p, start_points[po])
         
        max_dist_sorted = np.argsort(dist) 

        limit = np.min([int(len(bSpline.knotvector) * 0.5), np.shape(start_points)[0]])
        length_vector_min = np.inf
        for s_p in range(limit):
        # vector from point to closest point on spline
            t_old = t_start[max_dist_sorted[s_p]]
            vector = bSpline.evaluate_single(t_old) - p
            vector_length = np.linalg.norm(vector)
            if normals_to_inside:
                # check if normal and vector look in  approximately the same direction -> disabled
                if np.dot(vector / vector_length, normals[i] / np.linalg.norm(normals[i])) > -0 and vector_length < 1 * mean_distance_point_to_point: # 45 ° in both directions
                    # save all values and go to next point
                    if length_vector_min > vector_length:
                        length_vector_min = vector_length
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector

            else:
                # check if normal and vector look in  approximately the opposite direction -> disabled
                if np.dot(vector / vector_length, normals[i] / vector_length) < 0 and vector_length < 1 * mean_distance_point_to_point: # 45 ° in both directions
                    # save all values and go to next point
                    if length_vector_min > vector_length:
                        length_vector_min = vector_length
                        t_on_line[i] = t_old
                        vector_to_line[i] = vector
        angle = 0
        distance_multiplier = 1 
        while sum(vector_to_line[i]) == 0:
            angle += np.min([1, 0.1])
            for s_p in range(limit):
        # vector from point to closest point on spline
                t_old = t_start[max_dist_sorted[s_p]]
                vector = bSpline.evaluate_single(t_old) - p
                vector_length = np.linalg.norm(vector)
                if normals_to_inside:
                    # check if normal and vector look in  approximately the same direction -> disabled
                    if np.dot(vector / vector_length, normals[i] / np.linalg.norm(normals[i])) > -angle and vector_length < 1.5 * mean_distance_point_to_point: # 45 ° in both directions
                        # save all values and go to next point
                        if length_vector_min > vector_length:
                            length_vector_min = vector_length
                            t_on_line[i] = t_old
                            vector_to_line[i] = vector


                else:
                    # check if normal and vector look in  approximately the opposite direction -> disabled
                    if np.dot(vector / vector_length, normals[i] / np.linalg.norm(normals[i])) < angle and vector_length < 1.5 * mean_distance_point_to_point: # 45 ° in both directions
                        # save all values and go to next point
                        if length_vector_min > vector_length:
                            length_vector_min = vector_length
                            t_on_line[i] = t_old
                            vector_to_line[i] = vector

                #if angle == 1 and sum(vector_to_line[i]) == 0:
                    #angle = 0
                    #distance_mutliplier += 0.2
            
        random_offset = np.random.rand(1) * 0.05
        vector_smaller = bSpline.evaluate_single(np.max([0,t_on_line[i] - random_offset])) - p
        distance_smaller = np.linalg.norm(vector)
        vector_bigger = bSpline.evaluate_single(np.min([1,t_on_line[i] + random_offset])) - p
        distance_smaller = np.lina
        
    
    vector_to_line_distances = np.zeros(np.shape(vector_to_line)[0])
    vector_to_line_distances[:] = np.abs(np.linalg.norm(vector_to_line[:], axis=1))
    mean = np.mean(vector_to_line_distances)
    """
        vector_to_line: vector from pcd points to spline
        t_on_line: corresponding t values on spline [0,1]
        vector_to line_distances: lengths of vector_to_line vectors
    """
    return vector_to_line, t_on_line, vector_to_line_distances