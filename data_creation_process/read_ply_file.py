from plyfile import PlyData, PlyElement
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from sklearn.neighbors import NearestNeighbors

def main():
    current_dir = os.getcwd()
    path_1 = os.path.join("data_creation_process", "colon_part")
    path_2 = os.path.join("/home/yn86eniw/gaussian-splatting/data/sierra_colon_normals")
    path_3 = os.path.join("/home/yn86eniw/2d-gaussian-splatting/output/1e00cb9a-7/point_cloud/iteration_30000")
    path_4 = os.path.join("/home/yn86eniw/2d-gaussian-splatting/output/e7a01fbc-1/point_cloud/iteration_30000")
    

    path_5 = os.path.join("/home/yn86eniw/Documents/Bachelorarbeit2/data_creation_process/colon_part_render0/point_cloud/iteration_30000")
    path_6 = os.path.join("/home/yn86eniw/2d-gaussian-splatting/output/f367cc0c-e/point_cloud/iteration_30000")

    path_04_09_render0_0_317 = "/home/yn86eniw/2d-gaussian-splatting/output/7bd58b90-1/point_cloud/iteration_30000"
    path_x = "/home/yn86eniw/2d-gaussian-splatting/output/cc83a8ce-2/point_cloud/iteration_30000/"
    path_p = "/home/yn86eniw/2d-gaussian-splatting/output/40c271c0-5/point_cloud/iteration_50000"
    path_recons_11_9_1 = "/home/yn86eniw/2d-gaussian-splatting/output/perf_recon_11_9_1_scLR_0/point_cloud/iteration_30000"
    path_uncombined = path_recons_11_9_1
    path = os.path.join(path_uncombined, "point_cloud.ply")
    #is_3D = True
    is_3D = False
    export = True
    if is_3D: 
        pass
        """with open(path, 'rb') as file:
            plydata = PlyData.read(file)

            x = np.asarray(plydata['vertex'].data['x'])
            y = np.asarray(plydata['vertex'].data['y'])
            z = np.asarray(plydata['vertex'].data['z'])
            points = np.stack((x,y,z), axis=1)

            scale_x = np.asarray(plydata['vertex'].data['scale_0'])
            scale_y = np.asarray(plydata['vertex'].data['scale_1'])
            scale_z = np.asarray(plydata['vertex'].data['scale_2'])
            scaling = np.stack((scale_x, scale_y, scale_z), axis=1)
            #scaling = np.exp(scaling)
            #print(np.shape(exp_scaling))
            """"""smallest_sc_arg = np.ndarray.argmin(exp_scaling, axis=1)
            arg = np.zeros([np.shape(smallest_sc_arg)[0], 2], dtype=int)
            z = np.linspace(0, np.shape(smallest_sc_arg)[0]-1, np.shape(smallest_sc_arg)[0], dtype=int)
            arg[:,0] = z
            arg[:,1] = smallest_sc_arg[:]
            vector = np.zeros(np.shape(exp_scaling))
            vector[arg[:,0], arg[:,1]] = 1""""""
            #print(smallest_sc_arg[0:100])
            #print(vector[0:100])
            print("dann")

            rot_0 = np.asarray(plydata['vertex'].data['rot_0'])
            rot_1 = np.asarray(plydata['vertex'].data['rot_1'])
            rot_2 = np.asarray(plydata['vertex'].data['rot_2'])
            rot_3 = np.asarray(plydata['vertex'].data['rot_3'])
            rot_qu = np.stack((rot_0, rot_1, rot_2, rot_3), axis=1)

            r = R.from_quat(rot_qu)
            print(r.as_matrix()[0:2])
            r_np = r.as_matrix()

            eigen_val = scaling * scaling
            print(eigen_val)
            arg_max_sorted = np.argsort(eigen_val, axis=1)
            print(arg_max_sorted)
            x = np.linspace(0,np.shape(r_np)[0]-1,np.shape(r_np)[0],dtype=int)
            print(arg_max_sorted[0:2,1:].squeeze())
            #x = np.linspace(0,1,2,dtype=int)
            one_vectors_array = r_np[x,arg_max_sorted[:,1]]
            second_vectors_array = r_np[x,arg_max_sorted[:,2]]
            print(one_vectors_array)
            print(second_vectors_array)
            cross_array = np.cross(one_vectors_array, second_vectors_array)



            #r = rot_0
            x = rot_1
            y = rot_2
            z = rot_3

            """"""r = np.array([[1.0 - 2.0 * (y[:] * y[:] + z[:] * z[:]), 2.0 * (x[:] * y[:] - r[:] * z[:]), 2.0 * (x[:] * z[:] + r[:]* y[:])],
                            [2.0 * (x[:] * y[:] + r[:] * z[:]), 1.0 - 2.0 * (x[:] * x[:] + z[:] * z[:]), 2.0 * (y[:] * z[:] - r[:] * x[:])],
                            [2.0 * (x[:] * z[:] - r[:] * y[:]), 2.0 * (y[:] * z[:] + r[:] * x[:]), 1.0 - 2.0 * (x[:] * x[:] + y[:] * y[:])]])
            r = np.swapaxes(r, 0, 2)
            r = np.swapaxes(r, 1, 2)""""""

            s = np.zeros([np.shape(scaling)[0], 3,3])
            s[:,0,0] = scaling[:,0]
            s[:,1,1] = scaling[:,1]
            s[:,2,2] = scaling[:,2]
            print("s")
            print(s)
            print("cov")

            # Sigma = R * S * S^T * R^T
            #cov = np.matmul(r, np.matmul(s, np.matmul(np.swapaxes(s, 1, 2), np.swapaxes(r,1,2))))
            cov = np.matmul(r.as_matrix(), np.matmul(s, np.matmul(np.swapaxes(s, 1, 2), np.swapaxes(r.as_matrix(),1,2))))
            print(cov)
            eig_val, eig_vec = np.linalg.eig(cov[:])
            print("there")
            print(eig_val)
            print(np.shape(eig_val))
            print("here")
            print(eig_vec)
            print(np.shape(eig_vec))
            min_arg = np.argmin(eig_val, axis=1)
            #print(min_arg)

            arg_n = np.zeros([np.shape(eig_val)[0], 2], dtype=int)
            z = np.linspace(0, np.shape(eig_val)[0]-1, np.shape(eig_val)[0], dtype=int)
            arg_n[:,0] = z
            arg_n[:,1] = min_arg[:]

            print(eig_vec[0, arg_n[0,1]])
            print("normals")
            normals = cross_array#eig_vec[arg_n[:,0], arg_n[:, 1]]
            print(normals)
            

            """"""normals = np.reshape(np.matmul(r.as_matrix(),  np.reshape(vector, [*np.shape(vector), 1])), [np.shape(vector)[0], 3])
            print("here")
            print(np.shape(normals))
            #print(plydata['vertex'].data['x'])
            pcd = o3d.io.read_point_cloud(path)
            pcd.normals = o3d.utility.Vector3dVector(normals)""""""


            pcd = o3d.io.read_point_cloud(path)
            zeros = np.zeros(np.shape(normals))
            zeros[200:300] = normals[200:300]
            pcd.normals = o3d.utility.Vector3dVector(zeros)

            pcd_2 = o3d.io.read_point_cloud(path)
            pcd_2.estimate_normals()
            normals = np.asarray(pcd_2.normals)
            normals[500:] = 0
            pcd_2.normals = o3d.utility.Vector3dVector(normals)

            o3d.visualization.draw_geometries(
                [pcd],#, midline_pcd, min_distances_pcd],
                mesh_show_wireframe=True,
                mesh_show_back_face=True,
                point_show_normal=True,
            )"""
    else:
        with open(path, 'rb') as file:
            plydata = PlyData.read(file)

            x = np.asarray(plydata['vertex'].data['x'])
            y = np.asarray(plydata['vertex'].data['y'])
            z = np.asarray(plydata['vertex'].data['z'])
            points = np.stack((x,y,z), axis=1)

            scale_x = np.asarray(plydata['vertex'].data['scale_0'])
            scale_y = np.asarray(plydata['vertex'].data['scale_1'])
            scaling = np.stack((scale_x, scale_y), axis=1)

            rot_0 = np.asarray(plydata['vertex'].data['rot_0'])
            rot_1 = np.asarray(plydata['vertex'].data['rot_1'])
            rot_2 = np.asarray(plydata['vertex'].data['rot_2'])
            rot_3 = np.asarray(plydata['vertex'].data['rot_3'])
            rot_qu = np.stack((rot_0, rot_1, rot_2, rot_3), axis=1)

            r = R.from_quat(rot_qu)

            r_np = r.as_matrix()

            print(f"rot_: {r_np}")

            cross_array = r_np * np.array([0,0,1])


            """s = np.zeros([np.shape(scaling)[0], 3,3])
            s[:,0,0] = scaling[:,0]
            s[:,1,1] = scaling[:,1]
            
            cov = np.matmul(r.as_matrix(), np.matmul(s, np.matmul(np.swapaxes(s, 1, 2), np.swapaxes(r.as_matrix(),1,2))))
            
            eig_val, eig_vec = np.linalg.eig(cov[:])
            
            min_arg = np.argmin(eig_val, axis=1)
            print(f"scaling {s}")
            print(r.as_matrix())
            print(np.swapaxes(r.as_matrix(), 1, 2))
            print(f"cov{cov}")
            print("there")
            print(eig_val)
            print(np.shape(eig_val))
            print("here")
            print(eig_vec)
            print(np.shape(eig_vec))
            print(min_arg)

            arg_n = np.zeros([np.shape(eig_val)[0], 2], dtype=int)
            z = np.linspace(0, np.shape(eig_val)[0]-1, np.shape(eig_val)[0], dtype=int)
            arg_n[:,0] = z
            arg_n[:,1] = min_arg[:]"""

            #print(eig_vec[0, arg_n[1,1]])
            print("normals")
            normals = cross_array#eig_vec[arg_n[:,0], arg_n[:,1]]
            print(normals)

            pcd = o3d.io.read_point_cloud(path)
            zeros = np.zeros(np.shape(normals))
            zeros = normals
            #zeros[0:500] = normals[0:500]
            pcd.normals = o3d.utility.Vector3dVector(zeros)

            """pcd_2 = o3d.io.read_point_cloud(path)
            pcd_2.estimate_normals()
            normals = np.asarray(pcd_2.normals)
            normals[200:] = 0
            pcd_2.normals = o3d.utility.Vector3dVector(normals)"""

            o3d.visualization.draw_geometries(
                [pcd],#, midline_pcd, min_distances_pcd],
                mesh_show_wireframe=True,
                mesh_show_back_face=True,
                point_show_normal=True,
            )
            if export:
                plydata['vertex'].data['nx'] = normals[:,0]
                plydata['vertex'].data['ny'] = normals[:,1]
                plydata['vertex'].data['nz'] = normals[:,2]
                plydata.write(os.path.join(path_uncombined,"point_cloud_with_normals.ply"))
    


if __name__=="__main__": main()