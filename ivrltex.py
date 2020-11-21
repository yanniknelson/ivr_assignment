from sympy import *
init_printing(sue_unicode=True)
t1 = symbols('theta_1')
# t1 = 0
a1 = pi/2
rz1 = Matrix([[cos(t1 + pi/2), -sin(t1 + pi/2), 0, 0], [sin(t1 + pi/2), cos(t1 + pi/2), 0, 0],
              [0, 0, 1, 0], [0, 0, 0, 1]])
mz1 = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 2.5], [0, 0, 0, 1]])
mx1 = eye(4)
rx1 = Matrix([[1, 0, 0, 0], [0, cos(a1), -sin(a1), 0], [0, sin(a1), cos(a1), 0],
              [0, 0, 0, 1]])
A01 = rz1*mz1*mx1*rx1

t2 = symbols('theta_2')
a2 = pi/2
rz2 = Matrix([[cos(t2 + pi/2), -sin(t2 + pi/2), 0, 0], [sin(t2 + pi/2), cos(t2 + pi/2), 0, 0],
              [0, 0, 1, 0], [0, 0, 0, 1]])
mz2 = eye(4)
mx2 = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
rx2 = Matrix([[1, 0, 0, 0], [0, cos(a2), -sin(a2), 0], [0, sin(a2), cos(a2), 0],
              [0, 0, 0, 1]])
A12 = rz2*mz2*mx2*rx2

t3 = symbols('theta_3')
a3 = -pi/2
rz3 = Matrix([[cos(t3), -sin(t3), 0, 0], [sin(t3), cos(t3), 0, 0],
              [0, 0, 1, 0], [0, 0, 0, 1]])
mz3 = eye(4)
mx3 = Matrix([[1, 0, 0, 3.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
rx3 = Matrix([[1, 0, 0, 0], [0, cos(a3), -sin(a3), 0], [0, sin(a3), cos(a3), 0],
              [0, 0, 0, 1]])

A23 = rz3*mz3*mx3*rx3

t4 = symbols('theta_4')
a4 = 0
rz4 = Matrix([[cos(t4), -sin(t4), 0, 0], [sin(t4), cos(t4), 0, 0],
              [0, 0, 1, 0], [0, 0, 0, 1]])
mz4 = eye(4)
mx4 = Matrix([[1, 0, 0, 3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
rx4 = Matrix([[1, 0, 0, 0], [0, cos(a4), -sin(a4), 0], [0, sin(a4), cos(a4), 0],
              [0, 0, 0, 1]])

A34 = rz4*mz4*mx4*rx4

# pprint((A01), use_unicode=False)
# pprint((A01*A12), use_unicode=False)
# pprint((A01*A12*A23).col(3), use_unicode=False)
FK = A01*A12*A23*A34
# pprint(FK.col(3), use_unicode=False)
FK.col_del(1)
FK.col_del(0)
print(latex(FK))
# print(latex(FK))
# print("")
# print(latex(FK.col(3)))
# pprint((FK).col(3), use_unicode=False)
# print(latex(FK.col(3).row(0)))
# print(latex(FK.col(3).row(1)))
# print(latex(FK.col(3).row(2)))

# pprint(FK.col(3), use_unicode=False)
