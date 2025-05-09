points = [
    [-0.215, 0.1, 0.059],
    [-0.215, -0.1, 0.059],
    [-0.185, -0.135, 0.059],
    [0.095, -0.135, 0.059],
    [0.11, -0.08, 0.059],
    [0.11, 0.08, 0.059],
    [0.095, 0.135, 0.059],
    [-0.185, 0.135, 0.059],
    [-0.215, 0.1, 0.059],
    [-0.215, 0.1, 0.234],
    [-0.215, -0.1, 0.234],
    [-0.185, -0.135, 0.234],
    [0.095, -0.135, 0.234],
    [0.11, -0.08, 0.234],
    [0.11, 0.08, 0.234],
    [0.095, 0.135, 0.234],
    [-0.185, 0.135, 0.234],
    [-0.215, 0.1, 0.234],
]

# Width: 0.325 - 0.0175 * 4 - 0.0175 * 2 / sqrt(2) = 0.23
# Length: 0.27

# point [
#     0.046 -0.136 0.184 0.096 -0.136 0.184 0.134 -0.101 0.184 0.159 -0.054 0.184 0.168 0 0.184 0.159 0.054 0.184 0.134 0.101 0.184 0.096 0.136 0.184 0.046 0.136 0.184 0.043 -0.136 0.184 0.046 -0.136 0.234 0.096 -0.136 0.234 0.134 -0.101 0.234 0.159 -0.054 0.234 0.168 0 0.234 0.159 0.054 0.234 0.134 0.101 0.234 0.096 0.136 0.234 0.046 0.136 0.234 0.043 -0.136 0.234
# ]

# points = [
#     [0.046, -0.136, 0.184],
#     [0.096, -0.136, 0.184],
#     [0.134, -0.101, 0.184],
#     [0.159, -0.054, 0.184],
#     [0.168, 0, 0.184],
#     [0.159, 0.054, 0.184],
#     [0.134, 0.101, 0.184],
#     [0.096, 0.136, 0.184],
#     [0.046, 0.136, 0.184],
#     [0.043, -0.136, 0.184],
#     [0.046, -0.136, 0.234],
#     [0.096, -0.136, 0.234],
#     [0.134, -0.101, 0.234],
#     [0.159, -0.054, 0.234],
#     [0.168, 0, 0.234],
#     [0.159, 0.054, 0.234],
#     [0.134, 0.101, 0.234],
#     [0.096, 0.136, 0.234],
#     [0.046, 0.136, 0.234],
#     [0.043, -0.136, 0.234],
# ]

# Width: 0.125
# Lenght: 0.272

# point [
#     -0.135 0.136 0.184 -0.185 0.136 0.184 -0.223 0.101 0.184 -0.248 0.054 0.184 -0.257 0 0.184 -0.248 -0.054 0.184 -0.223 -0.101 0.184 -0.185 -0.136 0.184 -0.135 -0.136 0.184 -0.135 0.136 0.184 -0.135 0.136 0.234 -0.185 0.136 0.234 -0.223 0.101 0.234 -0.248 0.054 0.234 -0.257 0 0.234 -0.248 -0.054 0.234 -0.223 -0.101 0.234 -0.185 -0.136 0.234 -0.135 -0.136 0.234 -0.135 0.136 0.234
# ]

# points = [
#     [-0.135, 0.136, 0.184],
#     [-0.185, 0.136, 0.184],
#     [-0.223, 0.101, 0.184],
#     [-0.248, 0.054, 0.184],
#     [-0.257, 0, 0.184],
#     [-0.248, -0.054, 0.184],
#     [-0.223, -0.101, 0.184],
#     [-0.185, -0.136, 0.184],
#     [-0.135, -0.136, 0.184],
#     [-0.135, 0.136, 0.184],
#     [-0.135, 0.136, 0.234],
#     [-0.185, 0.136, 0.234],
#     [-0.223, 0.101, 0.234],
#     [-0.248, 0.054, 0.234],
#     [-0.257, 0, 0.234],
#     [-0.248, -0.054, 0.234],
#     [-0.223, -0.101, 0.234],
#     [-0.185, -0.136, 0.234],
#     [-0.135, -0.136, 0.234],
#     [-0.135, 0.136, 0.234],
# ]

# Width: 0.122
# Lenght: 0.272

# Total width: 0.23 + 0.125 + 0.122 = 0.477
# Total height: 0.175 + 0.05 + 0.05 = 0.275

x_values = [point[0] for point in points]
y_values = [point[1] for point in points]

width = max(x_values) - min(x_values)
length = max(y_values) - min(y_values)

print(f"Width: {width}")
print(f"Lenght: {length}")
