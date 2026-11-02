"""
探索器: modular-robotics の Pathfinder を外部プロセスとして呼ぶ.

探索器は find_moves(initial, goal) で {ID: 座標} と目標の座標の集合を受け取り、
手の列 [(ID, 移動量), ...] を返す。計画への変換と検査は commands.build_plan が受け持つ。
"""

import json
from pathlib import Path
import subprocess
import tempfile

from mocube_planner.model import add

TENSOR_PADDING = 3


def write_scenario(path, cells, static_cell, offset, axis_size):
    modules = [
        {'position': list(add(cell, offset)), 'static': cell == static_cell}
        for cell in cells
    ]
    path.write_text(json.dumps({
        'name': 'mocube', 'description': 'mocube_planner',
        'order': 3, 'axisSize': axis_size, 'tensorPadding': TENSOR_PADDING,
        'modules': modules,
    }), encoding='utf-8')


def parse_scen(text):
    """Pathfinder の出力を (ID -> 座標, [(ID, 移動量), ...]) にする."""
    cells, moves = {}, []
    for line in text.splitlines():
        fields = [f.strip() for f in line.split(',')]
        if len(fields) != 5:
            continue
        if fields[0].startswith('*'):
            moves.append((int(fields[0][1:]), tuple(int(v) for v in fields[2:])))
        elif fields[0].isdigit() and not moves:
            # 先頭の色の定義も同じ形だが、番号が同じモジュールの行で上書きされる
            cells[int(fields[0])] = tuple(int(v) for v in fields[2:])
    return cells, moves


def to_module_moves(initial, scen_cells, scen_moves):
    """.scen の ID を座標で照合し、手の列を初期形状の ID で表す."""
    if len(scen_cells) != len(initial):
        raise ValueError('Pathfinder の出力のモジュール数が初期形状と違います')
    scen_origin = min(scen_cells.values())
    origin = min(initial.values())
    by_cell = {cell: module for module, cell in initial.items()}
    scen_to_id = {
        scen_id: by_cell[add(tuple(a - b for a, b in zip(cell, scen_origin)), origin)]
        for scen_id, cell in scen_cells.items()
    }
    return [(scen_to_id[scen_id], step) for scen_id, step in scen_moves]


class PathfinderBackend:

    def __init__(self, binary, moves_dir, search='A*', timeout=None):
        """
        パラメータ.

        binary: Pathfinder の実行ファイル
        moves_dir: 動きの定義 (Mocube.json) のフォルダ
        search: 探索方法 (Pathfinder の -s)
        timeout: 探索の制限時間 [s]。None は無制限
        """
        self.binary = Path(binary).resolve()
        self.moves_dir = Path(moves_dir).resolve()
        self.search = search
        self.timeout = timeout
        if not self.binary.is_file():
            raise FileNotFoundError(
                f'Pathfinder がありません: {self.binary} '
                '(external/build_pathfinder.sh でビルドできます)')

    def find_moves(self, initial, goal):
        """手の列を探す. 見つからなければ RuntimeError."""
        goal = set(goal)
        static_cell = initial[0]
        if static_cell not in goal:
            raise ValueError(f'ID0 の位置 {static_cell} が目標形状に含まれていません')
        if len(goal) != len(initial):
            raise ValueError(f'モジュール数が違います: 初期 {len(initial)}, 目標 {len(goal)}')

        everything = [*initial.values(), *goal]
        low = [min(cell[axis] for cell in everything) for axis in range(3)]
        high = [max(cell[axis] for cell in everything) for axis in range(3)]
        offset = tuple(-v for v in low)
        axis_size = max(h - l + 1 for h, l in zip(high, low))

        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            write_scenario(directory / 'initial.json', list(initial.values()),
                           static_cell, offset, axis_size)
            write_scenario(directory / 'final.json', sorted(goal),
                           static_cell, offset, axis_size)
            output = directory / 'plan.scen'
            # Module Properties/ を相対パスで読むので、ビルドディレクトリの中で実行する
            result = subprocess.run(
                [self.binary, '-i', '-s', self.search, '-m', self.moves_dir,
                 '-I', directory / 'initial.json', '-F', directory / 'final.json',
                 '-e', output],
                cwd=self.binary.parent, capture_output=True, text=True, timeout=self.timeout)
            # 探し尽くして解がないときは終了コード 0 で空のファイルを出す。
            # 異常終了 (A* のヒューリスティックの例外など) は解がないことの証明ではない
            cells, moves = {}, []
            if result.returncode == 0 and output.exists():
                cells, moves = parse_scen(output.read_text(encoding='utf-8'))
            if not cells:
                raise RuntimeError(
                    f'Pathfinder が解を出せませんでした (終了コード {result.returncode})\n'
                    f'{result.stderr[-500:]}')
        return to_module_moves(initial, cells, moves)
