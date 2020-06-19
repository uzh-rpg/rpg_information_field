#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) Zichao Zhang, RPG, University of Zurich, Switzerland - All Rights Reserved
# You can contact the author at <zzhang at ifi dot uzh dot ch>
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import numpy as np
import plotly
import plotly.tools as tls

import tfs_utils as tu


def toColors(values):
    max_v = np.max(values)
    min_v = np.min(values)
    colors = values * 1.0 / (max_v - min_v) + \
        (-min_v) / (max_v - min_v)
    return colors


def plot3DPoints(ax, pts, alpha, scale=False):
    assert pts.shape[0] > 0
    assert pts.shape[1] == 3
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], alpha=alpha)
    if scale is True:
        min_v = np.min(pts)
        max_v = np.max(pts)
        ax.auto_scale_xyz([min_v, max_v], [min_v, max_v], [min_v, max_v])


def plotCoordinateFrame3d(T, ax, length=1.0, axes_c=['r', 'g', 'b']):
    t = T[0:3, 3]
    starts = np.tile(t, (3, 1))
    ends = np.zeros((3, 3))
    I33 = np.eye(3)
    for i in range(3):
        e = I33[i, :]
        ends[i, :] = tu.transformPt(T, e * length)
    ends = ends - starts
    ax.quiver(starts[:, 0], starts[:, 1], starts[:, 2],
              ends[:, 0], ends[:, 1], ends[:, 2],
              colors=axes_c, arrow_length_ratio=0.0)


def plotCoordinateFrame3dBatch(Ts, ax, length=1.0, axes_c=['r', 'g', 'b']):
    for T in Ts:
        plotCoordinateFrame3d(T, ax, length=length, axes_c=axes_c)


def plotTopView(poses, points, ax, name):
    plot3DPoints(ax, points, 0.1, scale=False)
    plotCoordinateFrame3dBatch(poses, ax, axes_c=['r', 'g', 'b'])
    ax.set_title(name)
    ax.view_init(azim=0, elev=90)


def multiPlot(values, axes, names):
    for v, ax, n in zip(values, axes, names):
        ax.plot(v, label=n)


def componentPlot(values, names):
    import matplotlib.pyplot as plt
    # each row is for a separate plot
    assert len(values) == len(names)
    dim = values[0].shape[0]
    fig, axes = plt.subplots(dim, 1)
    for v, n in zip(values, names):
        multiPlot(v, axes, [n]*dim)
    return fig


def sendToVisdom(mpl_fig, vis, fix_aspect=True, show_legend=False):
    plotly_fig = tls.mpl_to_plotly(mpl_fig, resize=True)
    if fix_aspect:
        plotly_fig.layout.yaxis.scaleanchor = 'x'
        plotly_fig.layout.yaxis.scaleratio = 1
    plotly_fig['layout'].pop('annotations', None)

    # Add legend, place it at the top right corner of the plot
    if show_legend:
        plotly_fig['layout']['showlegend'] = True
        plotly_fig['layout']['legend'] = {}
        plotly_fig['layout']['legend'].update({'x': 1.5, 'y': 0.5,
                                               'borderwidth': 1,
                                               'bgcolor': 'rgb(217,217,217)'})

    vis._send({'data': plotly_fig.data, 'layout': plotly_fig.layout})


def saveAsPlotly(mpl_fig, name):
    plotly_fig = tls.mpl_to_plotly(mpl_fig, resize=True)
    plotly.offline.plot(plotly_fig, filename=name, auto_open=False)


if __name__ == '__main__':
    pass
