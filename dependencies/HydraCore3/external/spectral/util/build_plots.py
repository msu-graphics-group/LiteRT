#!/bin/python3

import numpy as np
import sys
import subprocess
from pathlib import Path
from os.path import join 

from plot import plot_spd

LUTS = [1, 2, 3, 4]

TARGETS = ["ff00ff", "ff77ff", "ffff00", "ffff77", "00ffff", "77ffff",
           "ff0000", "00ff00", "0000ff", "ababab", "000000", "ffffff",
           "a05ff1", "151515", "010101", "000001", "010000", "000100",
           "101010", "129841", "990000", "009900", "000099", "990099",
           "999900", "009999", "fffe83", "fffff0"]

def plot_for_lut(lut):
    if lut is None:
        lut_path = join("output", "f_emission_lut.eflf")
    else:
        lut_path = join("resources", f"f_emission_lut_{lut}.eflf")
    print(lut_path)
    for target in TARGETS:
        print(target)
        subprocess.run(["./build/exporter", target, lut_path])
        plot_spd(join("output", "spd", target + ".spd"), False, "{}_" + str(lut) + ".png")

def main():
    if len(sys.argv) == 1:
        plot_for_lut(None)
    else:
        plot_for_lut(int(sys.argv[1]))


if __name__ == "__main__":
    main()
