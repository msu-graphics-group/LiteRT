#!/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import sys
import subprocess
import cv2
from pathlib import Path
from os.path import join 
from skimage import io
from skimage.metrics import structural_similarity as ssim
from skimage.metrics import peak_signal_noise_ratio as psnr
from plot import diff_image, mean_deltaE



METHODS = ["glassner", "smits", "sigpoly"]
IMAGES = [join("input", "textures", "brushed_concrete_03_diff_4k.jpg"),
          join("input", "textures", "coast_sand_03_diff_2k.jpg"),
     #     join("input", "textures", "fabric_pattern_07_col_1_2k.png"),
          join("input", "textures", "gravel_floor_02_diff_4k.jpg"),
          join("input", "textures", "medieval_red_brick_diff_4k.jpg"),
          join("input", "textures", "painted_concrete_diff_4k.jpg"),
          join("input", "textures", "tree_bark_03_diff_4k.jpg"),
          join("input", "textures", "wood_cabinet_worn_long_diff_4k.jpg")]
TARGET_DIR = join("output", "comparsion", "comparsion", "textures")


def run_test(path: str):
    image_gt = cv2.imread(path)
    image_gt_gray = cv2.cvtColor(image_gt, cv2.COLOR_BGR2GRAY)
    for method in METHODS:
        print(path + " : " + method)
        f = open(join(TARGET_DIR, Path(path).stem + "_" + method + ".txt"), "w+")
        #subprocess.run(["./build/comparsion", method, "im", path], stdout=f)

        image = cv2.imread(join(TARGET_DIR, f"{Path(path).stem}_{method}.png"))

        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        print(f"SSIM: {ssim(image_gt_gray, image_gray)}", file=f)
        print(f"PSNR: {psnr(image_gt, image)}", file=f)
        deltae = mean_deltaE(image_gt, image)
        print(f"MSE-deltaE: {deltae[:4]}", file=f)

        diff_path = join(TARGET_DIR, f"{Path(path).stem}_{method}_diff.png")
        #cv2.imwrite(diff_path, diff_image(deltae[4]))
        diff_image(deltae[4], diff_path)
        #plt.imsave(diff_path, deltae[4], cmap='inferno')
        

def main():
    if len(sys.argv) == 1:
        for img in IMAGES:
            run_test(img)
    else:
        run_test(sys.argv[1])


if __name__ == "__main__":
    main()
