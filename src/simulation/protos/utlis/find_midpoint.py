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


# translation 0.164 0.0 0.209
# rotation 0.643823 -0.056412  0.592334  0
# translation 0.147 -0.077 0.209
# rotation 0.935067 -0.250649 -0.250649 1.637883
q_1 = np.array([0.643823, -0.056412, 0.592334, 0])
p_1 = np.array([0.164, 0.0, 0.209])
q_2 = np.array([0.935067, -0.250649, -0.250649, 1.637883])
p_2 = np.array([0.147, -0.077, 0.209])
t = 0.5
# print with 6 decimal places and no scientific notation
print(np.array2string(slerp(q_1, q_2, t), precision=6, suppress_small=True))


# calculate the average
avg = (p_1 + p_2) / 2
# print with 3 decimal places and no scientific notation
print(np.array2string(avg, precision=3, suppress_small=True))
