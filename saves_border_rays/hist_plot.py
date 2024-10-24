import matplotlib.pyplot as plt
import numpy as np


def full_plot(title: str, arr: np.ndarray, fname: str):
    fig, (ax1, ax2) = plt.subplots(1, 2)
    fig.suptitle(title)
    fig.set_size_inches(13,4)

    xlim_l = -1
    xlim_h = arr.shape[0]+5
    ylim_l = 0
    ylim_h = int(np.max(arr))
    ylim_delim = 10
    if ylim_h > 100:
        ylim_delim = 100
    if ylim_h > 1000:
        ylim_delim = 1000
    ylim_h += ylim_delim - ylim_h % ylim_delim

    ax1.bar(range(0, arr.shape[0]), arr)
    ax1.set_ylabel("number of rays")
    ax1.set_xlabel("bin")
    ax1.set_xlim((xlim_l, xlim_h))
    ax1.set_ylim((ylim_l, ylim_h))


    arr = arr.cumsum()
    xlim_l = 0
    xlim_h = 1
    ylim_h *= arr.shape[0]
    # ylim_h = int(np.max(arr))
    # ylim_h += 1000 - ylim_h % 1000

    ax2.plot(np.arange(1, arr.shape[0]+1) / arr.shape[0], arr, marker='o' if arr.shape[0] < 100 else None)
    ax2.plot([0, 1], [0, arr[arr.shape[0]-1]])
    ax2.set_ylabel("number of rays (cumul)")
    ax2.set_xlabel("frac of relax_eps")
    ax2.set_xlim((xlim_l, xlim_h))
    ax2.set_ylim((ylim_l, ylim_h))
    plt.savefig(fname)
    plt.close()


def divide_into_bins(arr: np.ndarray, relax_eps: float, n_bins: int) -> np.ndarray:
    res = np.stack([ np.sum((arr >= (i * relax_eps / n_bins)) & (arr < ((i+1) * relax_eps / n_bins))) for i in range(n_bins) ])
    return res

def parse_data_file(n_bins: int, fpath: str = "./saves_border_rays/hist_data.txt"):
    with open(fpath) as hist_data_txt:
        relax_eps = 0.
        arr = np.zeros((n_bins,))
        hist_samples_count = 0

        while (line := hist_data_txt.readline()) != "":
            if line.startswith("Hist"):
                relax_eps, scene_ind, scene_step = line.split(' ')[1:]
                relax_eps = float(relax_eps)
                scene_ind = int(scene_ind)
                scene_step = int(scene_step)

            elif line == "\n":
                arr /= max(hist_samples_count, 1)

                title = "relax_eps = {}, {} bins, {} renders mean, {:.2f} rays, {:.2f} in the first bin".format(relax_eps, n_bins, hist_samples_count, arr.sum(), arr[0])
                full_plot(title, arr, "./saves_border_rays/Figure_{}_{}_{}_{}.png".format(scene_ind, scene_step, n_bins, relax_eps))
                hist_samples_count = 0
                arr = np.zeros((n_bins,))
            else:
                hist_samples_count += 1
                arr += divide_into_bins(np.array([ float(str_flt)  for str_flt in line.split(", ")[:-1] ]),
                                        relax_eps, n_bins)


if __name__ == "__main__":
    from sys import argv

    # n_bins = 10 if len(argv) <= 1 else int(argv[1])
    parse_data_file(10)
    parse_data_file(50)
