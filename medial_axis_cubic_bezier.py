import open3d as o3d
import os
from pc_skeletor import LBC
from pc_skeletor import utility
import numpy as np
import bezier


def extract_medial_axis(point_cloud):

    lbc = LBC(
        point_cloud=point_cloud,
        init_contraction=0.7,  # 0.9
        init_attraction=1,  # 2
        max_contraction=2048,
        max_attraction=1024,
        step_wise_contraction_amplification="auto",
        termination_ratio=0.03,
        max_iteration_steps=40,
        down_sample=0.008,
        filter_nb_neighbors=20,
        filter_std_ratio=2.0,
        debug=False,
        verbose=True,
    )
    lbc.extract_skeleton()
    lbc.extract_topology()
    return lbc


def get_end_point_line_pc(point_coordinates):
    # find the 2 points the furthest away from center of medial axis
    points_line = np.asarray(point_coordinates.points)
    center_line = point_coordinates.get_center()

    point_up_widest = 0
    point_down_widest = 0
    distance_point_up = 0
    distance_point_down = 0
    for x in points_line:
        distance = np.sqrt(np.dot(x - center_line, x - center_line))
        if distance > distance_point_up and sum(x - center_line) > 0:
            point_up_widest = x
            distance_point_up = distance
        if distance > distance_point_down and sum(x - center_line) < 0:
            distance_point_down = distance
            point_down_widest = x
    return np.array([point_up_widest, point_down_widest])

def get_curved_line_between_2_points_cubic(point1, point2, cloud1, cloud2, no_points=200, k1=1.5, k2 = 1.5): #1.5
    curved_line = np.ndarray([no_points,3])

    # calculate tangent of endpoint 1
    distance_min = 10
    dir_point1 = 0
    for i in cloud1.points:
        distance = np.sqrt((point1[0]-i[0])**2 + (point1[1]-i[1])**2 + (point1[2]-i[2])**2)
        if distance < distance_min and i is not point1:
            dir_point1 = i

    # calculate tangent of endpoint 2
    distance_min = 10
    dir_point2 = 0
    for i in cloud2.points:
        distance = np.sqrt((point2[0]-i[0])**2 + (point2[1]-i[1])**2 + (point2[2]-i[2])**2)
        if distance < distance_min and i is not point2:
            dir_point2 = i

    tangent1 = ((point1 - dir_point1)) / (np.linalg.norm((point1 - dir_point1)) + 0.001)
    tangent2 = ((point2 - dir_point2)) / (np.linalg.norm((point2 - dir_point2)) + 0.001)

    # cubic bezier curve between 2 endpoints
    b1 = tangent1 / k1 + point1
    b2 = tangent2 / k2 + point2

    for t in range(no_points):
        curved_line[t][0] = point1[0] * (1 - t/no_points)**3 + 3 * b1[0] * (t/no_points) *(1-t/no_points)**2 + 3 * b2[0] * (t/no_points)**2 * (1-t/no_points) + point2[0] * (t/no_points)**3
        curved_line[t][1] = point1[1] * (1 - t/no_points)**3 + 3 * b1[1] * (t/no_points) *(1-t/no_points)**2 + 3 * b2[1] * (t/no_points)**2 * (1-t/no_points) + point2[1] * (t/no_points)**3
        curved_line[t][2] = point1[2] * (1 - t/no_points)**3 + 3 * b1[2]* (t/no_points) *(1-t/no_points)**2 + 3 * b2[2] * (t/no_points)**2 * (1-t/no_points) + point2[2] * (t/no_points)**3

    # conversion to point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(curved_line)
    return pcd

def find_next_end_point(start_point, next_endpoints):
    # calculate distance between endpoints
    distance1 = np.sqrt((next_endpoints[0][0]-start_point[0])**2 + (next_endpoints[0][1]-start_point[1])**2 + (next_endpoints[0][2]-start_point[2])**2)
    distance2 = np.sqrt((next_endpoints[1][0]-start_point[0])**2 + (next_endpoints[1][1]-start_point[1])**2 + (next_endpoints[1][2]-start_point[2])**2)
    
    # set current endpoint and next start point
    if distance1 < distance2:
        next_end = next_endpoints[0]
        next_start = next_endpoints[1]
    else:
        next_end = next_endpoints[1]
        next_start = next_endpoints[0]
    return next_end, next_start

def main():
    # data
    path1 = "./data/colon_subdivided/colon-part_top_"
    path2 = "./data/colon_subdivided-more_divided/colon-part_top_"
    path3 = "./data/colon_subdivided-more_divided_2/colon-part_top_"
    list_paths = []
    for i in range(1,16):
        list_paths.append(os.path.join(os.getcwd(), f"{path3}{i}.ply"))

    #colon path
    path_c = os.path.join(os.getcwd(), "./data/Colon.ply")

    # load
    point_clouds = []
    for i in list_paths:
        point_clouds.append(o3d.io.read_point_cloud(i))
    # load colon model
    point_cloud_c = o3d.io.read_point_cloud(path_c)

    # extract medial axis
    medial_axis = []
    for i in point_clouds:
        medial_axis.append(extract_medial_axis(i))

    # find end points of lines
    end_points = np.ndarray([len(medial_axis),2,3])
    for i in range(len(medial_axis)):
        end_points[i] = get_end_point_line_pc(medial_axis[i].contracted_point_cloud)

    # cubic bezier between end points
    connection_lines = []
    for i in range(0,len(medial_axis)-1):
        if i == 0:
            start_point = end_points[i][1]
        end_point_plus_one, next_start = find_next_end_point(start_point, end_points[i+1])
        connection_lines.append(get_curved_line_between_2_points_cubic(start_point, end_point_plus_one, medial_axis[i].contracted_point_cloud, medial_axis[i+1].contracted_point_cloud))
        start_point = next_start

    """for i in range(len(medial_axis)-1,0, -1):
        if i == len(medial_axis)-1:
            start_point = end_points[i][1]
        end_point_plus_one, next_start = find_next_end_point(start_point, end_points[i-1])
        connection_lines.append(get_curved_line_between_2_points_cubic(start_point, end_point_plus_one, medial_axis[i].contracted_point_cloud, medial_axis[i-1].contracted_point_cloud))
        start_point = next_start"""


    # concatenate the medial axis clouds
    coll_medial_axis = np.asarray(medial_axis[0].contracted_point_cloud.points)
    for i in range(0, len(medial_axis)):
        if i < len(medial_axis)-1:
            coll_medial_axis = np.concatenate([coll_medial_axis, np.asarray(connection_lines[i].points)], axis=0)
        coll_medial_axis = np.concatenate([coll_medial_axis, np.asarray(medial_axis[i].contracted_point_cloud.points)],axis=0)


    coll_medial_axis_c = o3d.geometry.PointCloud()
    coll_medial_axis_c = o3d.geometry.PointCloud.paint_uniform_color(coll_medial_axis_c, np.array([1,1,0]))
    coll_medial_axis_c.points = o3d.utility.Vector3dVector(coll_medial_axis)

    o3d.io.write_point_cloud("./output/medial_axis.ply", coll_medial_axis_c)

    # visualize
    utility.visualize([point_cloud_c, coll_medial_axis_c], background_color=(1, 1, 1),
        point_size=3,
        line_width=1.0,)

if __name__=="__main__": main()