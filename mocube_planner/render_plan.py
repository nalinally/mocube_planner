"""計画の JSON を GIF アニメーションにする. ID0 は赤、動くモジュールは橙で描く."""

import argparse
import json

from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

CUBE_FACES = [
    [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)], [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)],
    [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], [(0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1)],
    [(0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1)], [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)],
]
SUBSTEPS = 6


def frames(plan):
    positions = {int(i): np.array(c, float) for i, c in plan['initial'].items()}
    yield dict(positions), None, 'initial'
    for k, step in enumerate(plan['steps'], 1):
        module = step['module']
        source, target = np.array(step['from'], float), np.array(step['to'], float)
        for t in np.linspace(0, 1, SUBSTEPS):
            position = source + t * (target - source)
            yield {**positions, module: position}, module, f'move {k}: ID{module}'
        positions[module] = target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('commands', help='plan コマンドが出力した計画の JSON')
    parser.add_argument('-o', '--output', required=True, help='GIF の出力先')
    args = parser.parse_args()

    with open(args.commands, encoding='utf-8') as f:
        all_frames = list(frames(json.load(f)))
    all_frames += [all_frames[-1]] * 8
    cells = np.array([list(p) for frame in all_frames for p in frame[0].values()])
    low, high = cells.min(axis=0), cells.max(axis=0) + 1

    figure = plt.figure(figsize=(5, 5))
    axes = figure.add_subplot(projection='3d')

    def draw(index):
        positions, moving, title = all_frames[index]
        axes.clear()
        for module, position in positions.items():
            color = '#d62728' if module == 0 else '#ff9f1c' if module == moving else '#8ecae6'
            polygons = [[position + np.array(v) for v in face] for face in CUBE_FACES]
            axes.add_collection3d(Poly3DCollection(
                polygons, facecolor=color, edgecolor='#333333', linewidth=0.5, alpha=0.9))
        axes.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]),
                 xlabel='x', ylabel='y', zlabel='z', title=title)
        axes.set_box_aspect(high - low)

    animation = FuncAnimation(figure, draw, frames=len(all_frames))
    animation.save(args.output, writer=PillowWriter(fps=6))


if __name__ == '__main__':
    main()
