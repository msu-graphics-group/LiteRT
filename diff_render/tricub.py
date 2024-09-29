import numpy as np


def cubic_interpolate(p, x):
    """
    Perform cubic interpolation on 4 points p0, p1, p2, p3 using Catmull-Rom spline.
    :param p: List of 4 points
    :param x: Position to interpolate (0 <= x <= 1)
    :return: Interpolated value
    """
    return p[1] + 0.5 * x * (p[2] - p[0] + x * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3] + x * (3.0 * (p[1] - p[2]) + p[3] - p[0])))


def tricubic_interpolate(grid, x, y, z):
    """
    Perform tricubic interpolation on a 4x4x4 grid of voxel values.
    :param grid: 4x4x4 numpy array of voxel values
    :param x: Position to interpolate along the x-axis (0 <= x <= 1)
    :param y: Position to interpolate along the y-axis (0 <= y <= 1)
    :param z: Position to interpolate along the z-axis (0 <= z <= 1)
    :return: Interpolated value
    """
    # Interpolate along x for each yz plane
    values_yz = np.zeros((4, 4))
    for j in range(4):
        for k in range(4):
            values_yz[j, k] = cubic_interpolate(grid[:, j, k], x)

    # Interpolate along y for the intermediate x-interpolated values
    values_z = np.zeros(4)
    for k in range(4):
        values_z[k] = cubic_interpolate(values_yz[:, k], y)

    # Interpolate along z for the final result
    result = cubic_interpolate(values_z, z)

    return result


# Example usage:
# Define a 4x4x4 grid of voxel values (this would usually come from your data)
grid = np.array(
    [
        [
            [0.1, 0.2, 0.3, 0.4],
            [0.2, 0.3, 0.4, 0.5],
            [0.3, 0.4, 0.5, 0.6],
            [0.4, 0.5, 0.6, 0.7],
        ],
        [
            [0.2, 0.3, 0.4, 0.5],
            [0.3, 0.4, 0.5, 0.6],
            [0.4, 0.5, 0.6, 0.7],
            [0.5, 0.6, 0.7, 0.8],
        ],
        [
            [0.3, 0.4, 0.5, 0.6],
            [0.4, 0.5, 0.6, 0.7],
            [0.5, 0.6, 0.7, 0.8],
            [0.6, 0.7, 0.8, 0.9],
        ],
        [
            [0.4, 0.5, 0.6, 0.7],
            [0.5, 0.6, 0.7, 0.8],
            [0.6, 0.7, 0.8, 0.9],
            [0.7, 0.8, 0.9, 1.0],
        ],
    ]
)

print(grid[0,:,:].shape)

# Interpolate at position (0.5, 0.5, 0.5) inside the grid
x, y, z = 0.5, 0.5, 0.5
interpolated_value = tricubic_interpolate(grid, x, y, z)
print(f"Interpolated value at ({x}, {y}, {z}): {interpolated_value}")
