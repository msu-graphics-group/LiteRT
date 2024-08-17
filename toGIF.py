import imageio.v3 as iio
import numpy as np
from os import listdir


def make_a_gif(dirpath: str, gifpath: str, ind: int) -> None:
    def keyfunc(fpath: str) -> int: return int(fpath[fpath.find("_") + 1 : fpath.rfind("_")])

    image_filenames = [ filename for filename in listdir(dirpath) if filename.endswith(f"_{ind}.bmp") ]
    image_filenames.sort(key= keyfunc)

    images = np.stack([ iio.imread(dirpath + "/" + filename) for filename in image_filenames ])
    iio.imwrite(gifpath + f"/anim_{ind}.gif", images, extension= ".gif", duration= 0.2)


if __name__ == "__main__":
    for d, directory in enumerate(("./saves/without_redist", "./saves/with_redist")):
        for i in range(16):
            make_a_gif(directory, f"./saves/{d}", i)
