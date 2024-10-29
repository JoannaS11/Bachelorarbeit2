import open3d as o3d
import os
import numpy as np
from datetime import datetime
from sklearn.neighbors import NearestNeighbors
from tqdm.autonotebook import tqdm


def export_pcd_as_ply(pcd, output_folder, output_name_without_ply, dir_name=None):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    if dir_name != None:
        o3d.io.write_point_cloud(
            os.path.join(
                os.getcwd(),
                output_folder,
                dir_name,
                f"{date_time}_{output_name_without_ply}.ply",
            ),
            pcd,
        )
    else:
        o3d.io.write_point_cloud(
            os.path.join(
                os.getcwd(), output_folder, f"{date_time}_{output_name_without_ply}.ply"
            ),
            pcd,
        )


def get_big_line_pointcloud(pcd, zyl_points, zyl_normals, mini_residual):
    # initialization of arrays
    midpoints = np.full(
        [np.shape(zyl_points)[0] // mini_residual + 1, 3], [-999.0, -999.0, -999.0]
    )
    mid_p_dist = np.full([np.shape(zyl_points)[0] // mini_residual + 1], [-999.0])

    # Calculate threshold distance in which to search for points which are close to the line
    nbrs = NearestNeighbors(n_neighbors=4, algorithm="ball_tree").fit(zyl_points)
    distances, indices = nbrs.kneighbors(zyl_points)
    mean_distance_point_to_line = np.sum(distances) / (np.shape(distances)[0] * 3)

    # iterate over every mini_residual point of pcd
    for line_start in tqdm(
        range(0, np.shape(zyl_points)[0], mini_residual),
        desc="Compute more centralized point cloud:",
    ):

        # calculate distance from points to line(startpoint + normal)
        distance = np.ndarray(np.shape(zyl_points)[0])
        tmp = np.cross(
            (zyl_points[:] - zyl_points[line_start]), zyl_normals[line_start]
        )
        for x in range(np.shape(zyl_points)[0]):
            distance[x] = np.sqrt(
                tmp[x][0] ** 2 + tmp[x][1] ** 2 + tmp[x][2] ** 2
            )

        # get distance indices which are within threshold from line
        distance[line_start] = 100
        pos_min = np.argwhere(abs(distance) <= mean_distance_point_to_line)

        # calculate distance from each of the points to startpoint
        points_min = zyl_points[pos_min[:]]
        vector_min = points_min[:] - zyl_points[line_start]
        distance_min = np.ndarray([np.shape(points_min)[0], 1])
        for i in range(np.shape(distance_min)[0]):
            distance_min[i] = np.sqrt(
                np.dot(
                    np.reshape(vector_min[i][0], [1, 3]),
                    np.reshape(vector_min[i][0], [3, 1]),
                )
            )

        # sort distances(from points to start point)
        sort_indices = np.argsort(distance_min, axis=0)
        pos_min = pos_min[sort_indices[:]]

        # find the corresponding point
        x = 0
        for index in pos_min:
            # check that point is not behind the point
            if (np.dot(zyl_points[index] - zyl_points[line_start], -(zyl_normals[line_start])) < 0):
                # check that normals look in contrary directions
                if np.dot(zyl_normals[index], zyl_normals[line_start]) < 0:
                    mid_p_dist[line_start // mini_residual] = distance_min[
                        sort_indices[x]
                    ]
                    midpoints[line_start // mini_residual] = (
                        zyl_points[line_start]
                        + (zyl_points[index] - zyl_points[line_start]) * 0.5
                    )
                    break

            x += 1

    # remove not used elements in mid_p_dist array
    mid_p_dist_arg_999 = np.argwhere(mid_p_dist == [-999])
    mid_p_dist = np.delete(mid_p_dist, mid_p_dist_arg_999, axis=0)
    midpoints = np.delete(midpoints, mid_p_dist_arg_999, axis=0)

    # discard too small and too big distances (mean +- 2 * sigma)
    mean_dist = np.mean(mid_p_dist)
    std_dist = np.std(mid_p_dist)
    if std_dist > 0:
        mid_p_dist_arg_too_small = np.argwhere(mid_p_dist < [mean_dist - 2 * std_dist])
        mid_p_dist_arg_too_big = np.argwhere(mid_p_dist > [mean_dist + 2 * std_dist])
        midpoints = np.delete(
            midpoints, np.r_[mid_p_dist_arg_too_small, mid_p_dist_arg_too_big], axis=0
        )

    # convert to pcd
    line = o3d.geometry.PointCloud()
    line.points = o3d.utility.Vector3dVector(midpoints)

    # second pcd without outliers
    line_without_outlier, _ = line.remove_statistical_outlier(
        nb_neighbors=4, std_ratio=2.0
    )

    return line, line_without_outlier, mean_dist, mean_distance_point_to_line
