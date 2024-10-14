import matplotlib.pyplot as plt
import numpy as np


def bar_plot(title: str, arr: np.ndarray, ylim: tuple[float, float]):
    plt.title(title)
    plt.bar(range(arr.shape[0]), arr)
    plt.ylabel("number of rays")
    plt.xlabel("bin")
    plt.ylim(ylim)
    plt.show()


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
                relax_eps = float(line.split(' ')[1])
            elif line == "\n":
                title = "relax_eps = {}, {} bins, {} renders mean".format(relax_eps, n_bins, hist_samples_count)

                arr /= max(hist_samples_count, 1)

                ylim_l = int(np.min(arr)) - 10
                ylim_l -= ylim_l % 50
                ylim_h = int(np.max(arr))
                ylim_h += 100 - ylim_h % 100
                bar_plot(title, arr, (float(ylim_l), float(ylim_h)))
            else:
                hist_samples_count += 1
                arr += divide_into_bins(np.array([ float(str_flt)  for str_flt in line.split(", ")[:-1] ]),
                                        relax_eps, n_bins)


if __name__ == "__main__":
    from sys import argv

    n_bins = 10 if len(argv) <= 1 else int(argv[1])
    parse_data_file(n_bins)