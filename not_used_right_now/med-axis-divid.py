import open3d as o3d
import os
from pc_skeletor import LBC
from pc_skeletor import utility
import numpy as np
import bezier


def extract_medial_axis(point_cloud):

    lbc = LBC(
        point_cloud=point_cloud,
        init_contraction=0.5,  # 0.9
        init_attraction=1,  # 2
        max_contraction=2048,
        max_attraction=1024,
        step_wise_contraction_amplification="auto",
        termination_ratio=0.003,
        max_iteration_steps=40,
        down_sample=0.0008,
        filter_nb_neighbors=20,
        filter_std_ratio=2.0,
        debug=False,
        verbose=False,
    )
    lbc.extract_skeleton()
    lbc.extract_topology()

    return lbc


def get_end_point_line_pc(point_coordinates):
    points_line = np.asarray(point_coordinates.points)
    center_line = point_coordinates.get_center()
    print(f"!!!!!!!!!!!!!!!!!center: {center_line}")
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

    """for x in points_line:
              distance = np.sqrt(np.dot(x - center_line, x - center_line))
              if distance > distance_point_up:
                     if distance < distance_point_down:
                            point_down_widest = point_up_widest
                            distance_point_down = distance_point_up
                            point_up_widest = x
                            distance_point_up = distance
                     else:
                            distance_point_down = distance
                            point_down_widest = x"""
    #print(f"asdtr: {points_line[0:6]} and points_line[1]")
    """print(
        f"!!!!!!!!!!!!!!!!!!!!!!!!!!points : {point_up_widest} and {point_down_widest}"
    )"""
    return point_up_widest, point_down_widest


def straight_line_between_2_points(point1, point2, no_points=40):
    startpoint = point1
    m = (point2 - point1) / no_points
    line_pc_array = np.ndarray([no_points, 3])
    for x in range(0, no_points):
        line_pc_array[x] = startpoint + m * x
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(line_pc_array)
    return pcd



def get_curved_line_between_2_points(point1, point2, cloud1, cloud2, no_points=60):
    curved_line = np.ndarray([no_points,3])
    # get centers medial axis or point cloud ?????????????????
    center_1 = cloud1.get_center()
    center_2 = cloud2.get_center()

    # calculate angle bisector of the two vectors of the 2 medial axes
    angle_half = ((point2-center_2) + (point1-center_1)) / 2

    # calculate midpoint of line between 2 end points
    mid_point_on_line = point1 + (point2 - point1) / 2

    # find point on pointcloud which is closest to line: midpoint + x * angle_bisector -->direction!!!!!!!!!!!!!!!!!!?
    A = np.concatenate((np.reshape(mid_point_on_line,[3,1]), np.reshape(angle_half,[3,1])), axis = 1)
    point_huelle = point1
    min_residual = 100 # willkürlich gewählt
    for x in cloud1.points:
        if np.linalg.lstsq(A ,x)[1] < min_residual:
            min_residual = np.linalg.lstsq(A ,x)[1]
            point_huelle = x            

    # calculate midpoint of bezier curve
    mid_tmp = mid_point_on_line + (angle_half/np.linalg.norm(angle_half)) * (np.linalg.norm(point_huelle-mid_point_on_line))

    # generate bezier curve between 2 end points
    for t in range(0,no_points):
       curved_line[t] = (1-t/no_points)**2 * point1 + 2 * (1 - t/no_points) * (t/no_points) * mid_tmp + (t/no_points)**2 * point2

    # convert numpy to pointcloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(curved_line)

    return pcd

# data
list_path = ["./data/zylinder.ply", 
"./data/Colon.ply", 
"./data/colon-part1.ply","./data/colon-part2.ply","./data/colon-part3.ply"
]
path_z = os.path.join(os.getcwd(), "./data/zylinder.ply")
path_s = os.path.join(os.getcwd(), "./data/Colon.ply")
path_1 = os.path.join(os.getcwd(), "./data/colon-part1.ply")
path_2 = os.path.join(os.getcwd(), "./data/colon-part2.ply")
path_3 = os.path.join(os.getcwd(), "./data/colon-part3.ply")
path_4 = os.path.join(os.getcwd(), "./data/colon-part4.ply")
path_5_c = os.path.join(os.getcwd(), "./data/colon-part5_curved.ply")
"""k = 0
point_clouds = []
for iter in list_path:
    path = os.path.join(os.getcwd(), iter)
    point_clouds.append(o3d.io.read_point_cloud(path))"""

# load
colon = o3d.io.read_point_cloud(path_s)
pcd1 = o3d.io.read_point_cloud(path_1)
pcd2 = o3d.io.read_point_cloud(path_2)
pcd3 = o3d.io.read_point_cloud(path_3)
pcd4 = o3d.io.read_point_cloud(path_4)
pcd5_c = o3d.io.read_point_cloud(path_5_c)

# extract lines
lbc1 = extract_medial_axis(pcd1)
lbc2 = extract_medial_axis(pcd2)
lbc3 = extract_medial_axis(pcd3)
lbc4 = extract_medial_axis(pcd4)
lbc5_c = extract_medial_axis(pcd5_c)

# find end points of lines
point_up_widest_3, point_down_widest_3 = get_end_point_line_pc(
    lbc3.contracted_point_cloud
)
point_up_widest_1, point_down_widest_1 = get_end_point_line_pc(
    lbc1.contracted_point_cloud
)
point_up_widest_2, point_down_widest_2 = get_end_point_line_pc(
    lbc2.contracted_point_cloud
)
point_up_widest_5, point_down_widest_5 = get_end_point_line_pc(
    lbc5_c.contracted_point_cloud
)

# straight line between 2 points
line_pc_3 = straight_line_between_2_points(point_down_widest_3, point_down_widest_2)

# find bezier curve between 2 points
line_pc_3_c = get_curved_line_between_2_points(point_down_widest_3, point_down_widest_2, lbc2.pcd,
        lbc3.pcd, no_points=60)
line_pc_5_c = get_curved_line_between_2_points(point_down_widest_1, point_down_widest_5, lbc1.pcd,
        lbc5_c.pcd)
line_pc_2_c = get_curved_line_between_2_points(point_up_widest_2, point_up_widest_1, lbc1.pcd,
        lbc2.pcd,)
# l = bezier.BezierSegment([np.array[point_2widest_distance_3, point_widest_distance_2],3])
# k = bezier.find_control_points([np.array[point_2widest_distance_3, point_widest_distance_2],3])


"""points_line_3 = np.asarray(lbc3.contracted_point_cloud.points)
center_line_3 = lbc3.contracted_point_cloud.get_center()
point_widest_distance_3 = 0
point_2widest_distance_3 = 0
distance_point_up = 0
distance_point_down = 0
for x in points_line_3:
       distance = np.linalg.norm(x - center_line_3)
       if distance > distance_point_up:
              if distance > distance_point_down:
                     point_2widest_distance_3 = point_widest_distance_3
                     distance_point_up = distance_point_down
                     point_widest_distance_3 = x
                     distance_point_down = distance
              else:
                     distance_point_up = distance
                     point_2widest_distance_3 = x
print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!points : {point_2widest_distance_3} and {point_2widest_distance_3}")
min_box_3 = lbc3.contracted_point_cloud.get_min_bound()
max_box_3 = lbc3.contracted_point_cloud.get_max_bound()
print(f"min bounding box {min_box_3} and {max_box_3}")"""

# visualize
utility.visualize(
    [
        lbc1.pcd,
        line_pc_3,
        line_pc_3_c,
        line_pc_5_c,
        line_pc_2_c,
        lbc2.pcd,
        lbc3.pcd,
        lbc5_c.pcd,
        lbc5_c.contracted_point_cloud,
        lbc1.contracted_point_cloud,
        lbc2.contracted_point_cloud,
        lbc3.contracted_point_cloud,
    ],
    background_color=(1, 1, 1),
    point_size=5,
    line_width=1.0,
)
# lbc1.visualize()
# lbc2.visualize()
# lbc3.visualize()

# lbc1.export_results('./output')
# path_o = "./output/02_skeleton_LBC.ply"
"""lbc.animate(init_rot=np.asarray([[1, 0, 0], [0, 0, 1], [0, 1, 0]]),
            steps=300,
            output='./output')"""

print("visualizing the mesh using open3D")

"""mesh2 = o3d.io.read_point_cloud(path_o)
#print(np.asarray(mesh.vertex_normals))
mesh = o3d.io.read_point_cloud(path_s)
#print(np.asarray(mesh.vertices))
print("triangles")
#print(np.asarray(mesh.triangles[0:20]))

#normals = o3d.geometry.Normal(mesh)
print(type(mesh.vertices[0]))
medial_axis = np.ndarray(len(mesh.vertices))
for x in range(0,len(mesh.vertices)):
       #print(x)
       mesh.vertices[x] = mesh.vertices[x] + 0.1 * mesh.vertex_normals[x]

       #print(z)


print("there")
print(np.asarray(mesh2.vertices))


o3d.visualization.draw_geometries([mesh2,mesh],

       mesh_show_wireframe = True,

mesh_show_back_face = True)"""
