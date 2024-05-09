import numpy as np

from plyfile import PlyData
from plyfile import PlyElement


class GaussianData:
    def __init__(
        self, mean, scale, rotation_axis, rotation_angle, diffuse_color, opacity
    ):
        self.mean = np.asarray(mean)
        self.scale = np.asarray(scale)
        self.diffuse_color = np.asarray(diffuse_color)
        self.opacity = opacity

        axis_multiplier = np.sin(rotation_angle * np.pi / 360.0)
        axis_multiplier = axis_multiplier / np.sqrt(
            np.sum(np.square(np.asarray(rotation_axis)))
        )

        self.quaternion = np.asarray(
            (
                np.cos(rotation_angle * np.pi / 360.0),
                rotation_axis[0] * axis_multiplier,
                rotation_axis[1] * axis_multiplier,
                rotation_axis[2] * axis_multiplier,
            )
        )

        self.quaternion = self.quaternion / np.linalg.norm(self.quaternion)


data = [
    GaussianData(
        mean=(0.0, 0.0, -5.0),
        scale=(0.1, 0.1, 0.1),
        rotation_axis=(0.0, 0.0, 1.0),
        rotation_angle=0.0,
        diffuse_color=(0.0, 1.0, 0.0),
        opacity=0.5,
    )
]

vertex = np.asarray(
    [
        (
            *item.mean,
            *(0.0, 0.0, 0.0),
            *((item.diffuse_color - 0.5) / 0.28209479177387814),
            *(0.0 for _ in range(45)),
            np.log(item.opacity) - np.log(1.0 - item.opacity),
            *np.log(item.scale),
            *item.quaternion,
            *(0.0 for _ in range(3)),
            0.0,
            0.0,
            *(0.0 for _ in range(3)),
            *(0.0 for _ in range(45)),
            0.0,
            *(0.0 for _ in range(15)),
        )
        for item in data
    ],
    dtype=[
        ("x", "f4"),
        ("y", "f4"),
        ("z", "f4"),
        ("nx", "f4"),
        ("ny", "f4"),
        ("nz", "f4"),
        *(("f_dc_{}".format(i), "f4") for i in range(3)),
        *(("f_rest_{}".format(i), "f4") for i in range(45)),
        ("opacity", "f4"),
        *(("scale_{}".format(i), "f4") for i in range(3)),
        *(("rot_{}".format(i), "f4") for i in range(4)),
        *(("base_color_{}".format(i), "f4") for i in range(3)),
        ("roughness", "f4"),
        ("metallic", "f4"),
        *(("incidents_dc_{}".format(i), "f4") for i in range(3)),
        *(("incidents_rest_{}".format(i), "f4") for i in range(45)),
        ("visibility_dc_0", "f4"),
        *(("visibility_rest_{}".format(i), "f4") for i in range(15)),
    ],
)

PlyData([PlyElement.describe(vertex, "vertex")]).write("simple.ply")
