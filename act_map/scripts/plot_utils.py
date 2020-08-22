#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np


def color_box(bp, color):
    elements = ['medians', 'boxes', 'caps', 'whiskers']
    # Iterate over each of the elements changing the color
    for elem in elements:
        [plt.setp(bp[elem][idx], color=color, linestyle='-', lw=1.0)
         for idx in range(len(bp[elem]))]
    return


def boxplot_compare(ax, xlabels,
                    data, data_labels, data_colors,
                    legend=True):
    n_data = len(data)
    n_xlabel = len(xlabels)
    leg_handles = []
    leg_labels = []
    idx = 0
    for idx, d in enumerate(data):
        # print("idx and d: {0} and {1}".format(idx, d))
        w = 1 / (1.5 * n_data + 4.0)
        widths = [w for pos in np.arange(n_xlabel)]
        positions = [pos - 0.5 + 1.5 * w + idx * w * 1.2
                     for pos in np.arange(n_xlabel)]
        # print("Positions: {0}".format(positions))
        bp = ax.boxplot(d, 0, '', positions=positions, widths=widths)
        color_box(bp, data_colors[idx])
        tmp, = plt.plot([1, 1], data_colors[idx])
        leg_handles.append(tmp)
        leg_labels.append(data_labels[idx])
        idx += 1
    ax.set_xticks(np.arange(n_xlabel))
    ax.set_xticklabels(xlabels)
    xlims = ax.get_xlim()
    ax.set_xlim([xlims[0]-0.1, xlims[1]-0.1])
    if legend:
        # ax.legend(leg_handles, leg_labels, bbox_to_anchor=(
            # 1.05, 1), loc=2, borderaxespad=0.)
        ax.legend(leg_handles, leg_labels, ncol=3)
    map(lambda x: x.set_visible(False), leg_handles)