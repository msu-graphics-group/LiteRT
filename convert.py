import json
import os

from tqdm import tqdm

import numpy as np


def convert(scene: str, template: str, dataset: str):
    file = "/home/raenchuk/research/datasets/{}/transforms_{}.json".format(
        scene, dataset
    )

    with open(file) as fp:
        data = json.load(fp)

    fov = data["camera_angle_x"] * 180 / np.pi
    c2ws = [
        np.array(data["frames"][i]["transform_matrix"])
        for i in range(100 if dataset == "train" else 200)
    ]

    for i, c2w in enumerate(c2ws):
        c2w[:3, 1:3] *= -1
        w2c = np.linalg.inv(c2w)

        R = np.identity(4)
        R[:3, :3] = w2c[:3, :3]

        T = w2c[:3, 3]
        T *= -1

        scene_data = template.format(
            scene,
            i if dataset == "train" else i + 100,
            scene,
            fov,
            " ".join(map(str, T.ravel())),
            " ".join(map(str, T[:2].ravel())),
            " ".join(map(str, R.ravel())),
        )

        path = os.path.join(
            "scenes",
            "03_gs_scenes",
            "{}_{}.xml".format(
                scene, str(i if dataset == "train" else i + 100).zfill(5)
            ),
        )

        with open(path, "w") as fp:
            fp.write(scene_data)


if __name__ == "__main__":
    template = """<!-- this scene is similar to the {} train/r_{} scene -->
<?xml version="1.0"?>
<textures_lib/>
<materials_lib/>
<geometry_lib>
  <gs points="../../../gaussian-splatting/output/{}/point_cloud/iteration_30000/point_cloud.ply"/>
</geometry_lib>
<lights_lib/>
<cam_lib>
  <camera>
    <fov>{}</fov>
    <nearClipPlane>0.01</nearClipPlane>
    <farClipPlane>100.0</farClipPlane>
    <position>{}</position>
    <look_at>{} 100.0</look_at>
    <up>0.0 -1.0 0.0</up>
  </camera>
</cam_lib>
<render_lib/>
<scenes>
  <scene>
    <instance matrix="{}"/>
  </scene>
</scenes>
"""

    scenes = "chair", "drums", "ficus", "hotdog", "lego", "materials", "mic", "ship"
    os.makedirs(os.path.join("scenes", "03_gs_scenes"), exist_ok=True)

    for scene in tqdm(scenes, total=len(scenes)):
        convert(
            scene=scene,
            template=template,
            dataset="train",
        )

        convert(
            scene=scene,
            template=template,
            dataset="test",
        )
