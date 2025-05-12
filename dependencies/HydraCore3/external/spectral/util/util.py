#!/bin/python3
import sys
from pathlib import Path
from os.path import join
import numpy as np

def load_spd(src_path: str):
    lst = []
    with open(src_path, "r") as f:
        for s in f:
            spl = s.split(' ')
            lst.append([float(spl[0]), float(spl[1])])
    return np.array(lst)

def save_spd(dest_path: str, spd: np.ndarray):
    with open(dest_path, "w") as f:
        for p in spd:
            print(f"{p[0]} {p[1]}", file=f)

def save_as_csv(dest_path: str, spd: np.ndarray):
    with open(dest_path, "w") as f:
        print(','.join(np.char.mod("%f", spd[:, 0])), file=f)
        print(','.join(np.char.mod("%f", spd[:, 1])), file=f)


def resample(spd: np.ndarray, scale: float):
    wl = np.arange(360, 831, 1)
    val = np.interp(wl, spd[:, 0] * scale, spd[:, 1], left=0, right=0)
    return np.stack((wl, val), axis=-1)


if __name__ == "__main__":
    if(sys.argv[1] == "spd2csv"):
       save_as_csv(sys.argv[3], load_spd(sys.argv[2]))
    if(sys.argv[1] == "resamplesc"):
        save_spd(sys.argv[3], resample(load_spd(sys.argv[2]), float(sys.argv[4])))
    if(sys.argv[1] == "resample"):
        save_spd(sys.argv[3], resample(load_spd(sys.argv[2]), 1.0))
    if(sys.argv[1] == "resample2csv"):
        save_as_csv(sys.argv[3], resample(load_spd(sys.argv[2])))

 
