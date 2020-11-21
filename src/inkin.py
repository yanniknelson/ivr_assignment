import numpy as np
def invkin(t1, t2, t3, t4):
    st1 = np.sin(t1)
    ct1 = np.cos(t1)
    st2 = np.sin(t2)
    ct2 = np.cos(t2)
    st3 = np.sin(t3)
    ct3 = np.cos(t3)
    st4 = np.sin(t4)
    ct4 = np.cos(t4)
    x = 3*(st1*st2*ct3 + st3*ct1)*ct4 + 3.5*st1*st2*ct3 + 3*st1*st4*ct2 + 3.5*st3*ct1
    y = 3*(st1*st3 - st2*ct1*ct3)*ct4 + 3.5*st1*st3 - 3.5*st2*ct1*ct3 - 3*st4*ct1*ct2
    z = -3*st2*st4 + 3*ct2*ct3*ct4 + 3.5*ct2*ct3 + 2.5
    return np.array([x,y,z])

print(invkin(0, 1.5, 1.5, 0))