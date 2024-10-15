import numpy as np
from scipy.spatial.transform import Rotation as R

def fct(x):
    return x + 5

rot_vec = np.array([0, np.pi, 0])
rot = R.from_rotvec(rot_vec)
l = rot.as_matrix()
m = rot.as_quat()

rot_vec_2 = np.array([0, np.pi / 4, 0])
rot_2 = R.from_rotvec(rot_vec_2)

l_2 = rot_2.as_matrix()
m_2 = rot_2.as_quat()

new_rot_m = np.matmul(l, l_2)
new_rot_q = np.add(m , m_2)
print(new_rot_m)

print(R.from_quat(new_rot_q).as_matrix())


x = np.array([1,2,6,-2])
k = np.zeros(np.shape(x))
l = np.fmax(x, 0)
m = np.where(x < 0,0,fct(x))
print(m)