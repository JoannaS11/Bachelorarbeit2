import numpy as np

def fct(x):
    return x + 5

x = np.array([1,2,6,-2])
k = np.zeros(np.shape(x))
l = np.fmax(x, 0)
m = np.where(x < 0,0,fct(x))
print(m)