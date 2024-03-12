import numpy as np


def slerp(q1, q2, t):
    """Spherical linear interpolation."""
    dot = np.dot(q1, q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > 0.9995:
        return q1 + t * (q2 - q1)
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0 * q1 + s1 * q2


# translation -0.253 0.027 0.209
# rotation -0.061629 -0.705763 -0.705763 -3.01849
# translation -0.253 -0.027 0.209
# rotation -0.061839 0.705753 0.705753 -3.018072
p_1 = np.array([-0.253, 0.027, 0.209])
q_1 = np.array([-0.061629, -0.705763, -0.705763, -3.01849])
p_2 = np.array([-0.253, -0.027, 0.209])
q_2 = np.array([-0.061839, 0.705753, 0.705753, -3.018072])
t = 0.5
# print with 6 decimal places and no scientific notation
print(np.array2string(slerp(q_1, q_2, t), precision=6, suppress_small=True))


# calculate the average
avg = (p_1 + p_2) / 2
# print with 3 decimal places and no scientific notation
print(np.array2string(avg, precision=3, suppress_small=True))


def multiply_quaternions(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


q1 = (1, 0, 0, 1.5708)
q2 = (0, 1, 0, 3.14159)
result = multiply_quaternions(q1, q2)
print(result)
