import sys
if len(sys.argv) < 4:
    print('Usage: python3 plot_turbine.py input_file.csv model_name save-format')
    print('save-format is optional, can be "png", "pdf", "svg", etc')
    exit(1)
else:
    csv_path = sys.argv[1]

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd
from matplotlib.ticker import LogLocator, FormatStrFormatter, ScalarFormatter

target_mode_name = sys.argv[2]
format = sys.argv[3]

df = pd.read_csv(csv_path)
# Strip whitespace
#df = df.map(lambda x: x.strip() if isinstance(x, str) else x)
df.columns = df.columns.str.strip()

df = df[(df.model_name == target_mode_name) & (df['type'] != 'MESH')]

# Здесь можно поставить font='Times New Roman' или любой другой
sns.set_theme(style='whitegrid', font_scale=1.3)


def normal_plot():
    plt.figure(figsize=(9, 6))
  
    ax = sns.lineplot(
        data=df,
        x='model_size(Mb)',
        y='psnr_average',
        hue='tag',
        marker='o',
        linewidth=2,
        palette='Set1',
    )
    ax.set_title('Quality versus model size for different model types')
    ax.legend(title='Model type', loc='lower right')
    ax.set_xlabel('Model size (Mb)')
    ax.set_ylabel('PSNR')

    ax.minorticks_on()
    return ax


def draw_plot(basename):
    plt.tight_layout()
    if format is None:
        plt.show()
    else:
        filename = f'plot_turbine_{basename}.{format}'
        print(f'Saving {filename}')
        plt.savefig(filename)


###############
# Normal plot #
###############
ax = normal_plot()
ax.grid(True, 'major', alpha=0.7, linewidth=0.8)
ax.grid(True, 'minor', alpha=0.3, linestyle='--', linewidth=0.5)

draw_plot('output1')

#############
# Log2 plot #
#############
ax = normal_plot()
ax.grid(True, 'major', alpha=0.7, linewidth=0.9)
ax.grid(True, 'minor', alpha=0.6, linestyle='--', linewidth=0.6)

ax.set_xscale('log', base=2)

# Remove scientific notation
major_formatter = FormatStrFormatter("%.2f")
ax.xaxis.set_major_formatter(major_formatter)

draw_plot('output2')

##############
# Log10 plot #
##############
ax = normal_plot()
ax.grid(True, 'major', alpha=0.7, linewidth=0.9)
ax.grid(True, 'minor', alpha=0.6, linestyle='--', linewidth=0.6)

ax.set_xscale('log')
ax.tick_params(which="both", bottom=True)

minor_locator = LogLocator(base=10,
                           subs=np.arange(0, 10, 2) * 0.1,
                           numticks=10)
ax.xaxis.set_minor_locator(minor_locator)
ax.xaxis.set_minor_formatter(FormatStrFormatter("%.1f"))

# Remove scientific notation
major_formatter = FormatStrFormatter("%.2f")
ax.xaxis.set_major_formatter(major_formatter)

draw_plot('output3')
