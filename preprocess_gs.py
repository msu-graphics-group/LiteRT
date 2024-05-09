import numpy as np

import argparse
import struct

from collections import defaultdict
from plyfile import PlyData
from tqdm import tqdm
from tqdm import trange


class AABB:
    def __init__(self, min_point, max_point, index=None):
        self.min_point = np.asarray(min_point)
        self.max_point = np.asarray(max_point)
        self.mid_point = (self.min_point + self.max_point) / 2
        self.index = index

    def contains(self, other):
        return np.all(self.min_point <= other.min_point) and np.all(
            self.max_point >= other.max_point
        )

    def intersects(self, other):
        return np.all(self.max_point >= other.min_point) and np.all(
            self.min_point <= other.max_point
        )


class OctreeNode:
    def __init__(self, boundary, depth, max_depth):
        assert depth > 0
        assert max_depth > 0

        self.boundary = boundary
        self.depth = depth
        self.max_depth = max_depth
        self.children = []
        self.leaves = []

    def subdivide(self):
        if self.depth >= self.max_depth or self.children:
            return False

        for dx, dy, dz in np.ndindex(2, 2, 2):
            min_point = self.boundary.min_point + np.asarray((dx, dy, dz)) * (
                self.boundary.mid_point - self.boundary.min_point
            )

            max_point = min_point + (self.boundary.mid_point - self.boundary.min_point)

            self.children.append(
                OctreeNode(
                    AABB(min_point, max_point),
                    depth=self.depth + 1,
                    max_depth=self.max_depth,
                )
            )

        return True

    def insert(self, leaf, force=False, max_leaf_depth=None):
        if not self.boundary.contains(leaf):
            if not self.boundary.intersects(leaf) or not force:
                return False

        if force and max_leaf_depth and self.depth >= max_leaf_depth:
            self.leaves.append(leaf)
            return True

        if self.depth >= self.max_depth:
            self.leaves.append(leaf)
            return True

        subdivided = False

        if not self.children:
            subdivided = self.subdivide()

        inserted = False

        for child in self.children:
            if child.insert(leaf, force=force, max_leaf_depth=max_leaf_depth):
                inserted = True

        if not inserted:
            self.leaves.append(leaf)

            if subdivided:
                self.children.clear()

        return True

    def collect_leaves(self):
        result = defaultdict(list)

        for leaf in self.leaves:
            result[self.boundary].append(leaf)

        for child in self.children:
            for boundary, leaves in child.collect_leaves().items():
                for leaf in leaves:
                    result[boundary].append(leaf)

        return result

    def calculate_max_leaf_depth(self):
        if not self.children:
            return self.depth

        return max(child.calculate_max_leaf_depth() for child in self.children)

    def propagate_leaves(self, max_leaf_depth, progress_bar=None):
        if progress_bar:
            progress_bar.update(1)

        if self.children and self.depth < max_leaf_depth:
            for child in self.children:
                for leaf in self.leaves:
                    if child.boundary.intersects(leaf):
                        child.insert(leaf, force=True, max_leaf_depth=max_leaf_depth)

            self.leaves.clear()

        for child in self.children:
            child.propagate_leaves(
                max_leaf_depth=max_leaf_depth, progress_bar=progress_bar
            )


def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q / np.linalg.norm(q)

    return np.asarray(
        (
            (
                1.0 - 2.0 * qy * qy - 2.0 * qz * qz,
                2.0 * qx * qy - 2.0 * qz * qw,
                2.0 * qx * qz + 2.0 * qy * qw,
            ),
            (
                2.0 * qx * qy + 2.0 * qz * qw,
                1.0 - 2.0 * qx * qx - 2.0 * qz * qz,
                2.0 * qy * qz - 2.0 * qx * qw,
            ),
            (
                2.0 * qx * qz - 2.0 * qy * qw,
                2.0 * qy * qz + 2.0 * qx * qw,
                1.0 - 2.0 * qx * qx - 2.0 * qy * qy,
            ),
        )
    )


def ellipsoid_bounding_box(position, scale, rotation):
    rotation_matrix = quaternion_to_rotation_matrix(rotation)

    min_x = position[0] - np.sqrt(np.sum(np.square(scale * rotation_matrix[0])))
    max_x = position[0] + np.sqrt(np.sum(np.square(scale * rotation_matrix[0])))

    min_y = position[1] - np.sqrt(np.sum(np.square(scale * rotation_matrix[1])))
    max_y = position[1] + np.sqrt(np.sum(np.square(scale * rotation_matrix[1])))

    min_z = position[2] - np.sqrt(np.sum(np.square(scale * rotation_matrix[2])))
    max_z = position[2] + np.sqrt(np.sum(np.square(scale * rotation_matrix[2])))

    return np.asarray((min_x, min_y, min_z)), np.asarray((max_x, max_y, max_z))


def write_to_file(minimum_box, maximum_box, data, filename):
    with open(filename, "wb") as file:
        file.write(struct.pack("6f", *minimum_box, *maximum_box))

        for item in data:
            floats, integers = item

            file.write(struct.pack("6fI", *floats, len(integers)))
            file.write(struct.pack(f"{len(integers)}I", *integers))


def main(input_path, output_path, max_depth):
    data = PlyData.read(input_path)

    xyz = np.stack(
        (
            np.asarray(data.elements[0]["x"]),
            np.asarray(data.elements[0]["y"]),
            np.asarray(data.elements[0]["z"]),
        ),
        axis=1,
    )

    scale_names = [
        p.name for p in data.elements[0].properties if p.name.startswith("scale_")
    ]

    rot_names = [
        p.name for p in data.elements[0].properties if p.name.startswith("rot")
    ]

    scale_names = sorted(scale_names, key=lambda x: int(x.split("_")[-1]))
    rot_names = sorted(rot_names, key=lambda x: int(x.split("_")[-1]))

    scales = np.zeros((xyz.shape[0], len(scale_names)))
    rots = np.zeros((xyz.shape[0], len(rot_names)))

    for idx, attr_name in enumerate(scale_names):
        scales[:, idx] = np.asarray(data.elements[0][attr_name])

    for idx, attr_name in enumerate(rot_names):
        rots[:, idx] = np.asarray(data.elements[0][attr_name])

    scales = np.exp(scales) * 3.0
    rots[:, 1] = -rots[:, 1]

    for i in range(rots.shape[0]):
        rots[i] = rots[i] / np.linalg.norm(rots[i])
        rots[i, 1:] = -rots[i, 1:]

    aabb_list = []

    for i in trange(xyz.shape[0], desc="Constructing bounding boxes"):
        aabb_list.append(
            AABB(*ellipsoid_bounding_box(xyz[i], scales[i], rots[i]), index=i)
        )

    minimum_box = np.asarray([i.min_point for i in aabb_list]).min(axis=0)
    maximum_box = np.asarray([i.max_point for i in aabb_list]).max(axis=0)

    print("Minimum box:", minimum_box)
    print("Maximum box:", maximum_box)

    octree_root = OctreeNode(
        AABB(minimum_box, maximum_box), depth=1, max_depth=max_depth,
    )

    for i, aabb in enumerate(
        tqdm(aabb_list, desc="Inserting bounding boxes to octree")
    ):
        octree_root.insert(aabb)

    leaves = octree_root.collect_leaves()
    output = []

    for key, item in leaves.items():
        output.append(((*key.min_point, *key.max_point), [i.index for i in item]))

    print("\nNodes amount:", len(output))
    print("Max bounding boxes inside node:", max([len(item[1]) for item in output]))
    print("Max leaf depth:", octree_root.calculate_max_leaf_depth(), end="\n\n")

    print("Propagating leaves (it takes some time)...")

    max_leaf_depth = octree_root.calculate_max_leaf_depth()

    progress_bar = tqdm(total=sum(8 ** i for i in range(max_depth)), unit="iteration")
    octree_root.propagate_leaves(
        max_leaf_depth=max_leaf_depth, progress_bar=progress_bar
    )
    progress_bar.close()

    print("Done!")

    leaves = octree_root.collect_leaves()
    output = []

    for key, item in leaves.items():
        output.append(((*key.min_point, *key.max_point), [i.index for i in item]))

    print("\nNodes amount:", len(output))
    print("Max bounding boxes inside node:", max([len(item[1]) for item in output]))
    print("Max leaf depth:", octree_root.calculate_max_leaf_depth())

    write_to_file(minimum_box, maximum_box, output, output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Octree construction and leaf propagation"
    )

    parser.add_argument("input_path", type=str, help="Path to the input PLY file")
    parser.add_argument("output_path", type=str, help="Path to the output binary file")
    parser.add_argument(
        "--max_depth", type=int, default=8, help="Maximum depth of the octree"
    )

    args = parser.parse_args()

    main(args.input_path, args.output_path, args.max_depth)
