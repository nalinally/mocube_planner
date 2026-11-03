"""
mocube の変形のモデル: 状態 (ID -> 格子座標) と、1 台を 1 マス動かす手の検査.

ROS に依存しないので単体テストできる。
ID0 は原点に固定で、シナリオファイルでは static のモジュールが ID0 になる。
"""

import json
from pathlib import Path

from mocube_estimator.geometry import contacts  # noqa: F401  (ここからも使えるようにする)

SIDES = tuple(
    tuple((sign if axis == i else 0) for i in range(3))
    for axis in range(3) for sign in (1, -1))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def load_scenario(path):
    """シナリオの JSON を {ID: 座標} で返す. static のモジュールが ID0 で、その座標が原点."""
    modules = json.loads(Path(path).read_text(encoding='utf-8'))['modules']
    modules.sort(key=lambda m: not m.get('static', False))
    if sum(m.get('static', False) for m in modules) != 1:
        raise ValueError('static のモジュール (ID0) はちょうど 1 台にしてください')
    origin = modules[0]['position']
    return {i: tuple(p - o for p, o in zip(m['position'], origin)) for i, m in enumerate(modules)}


def is_connected(cells):
    cells = set(cells)
    stack = [next(iter(cells))]
    seen = {stack[0]}
    while stack:
        cell = stack.pop()
        for side in SIDES:
            neighbor = add(cell, side)
            if neighbor in cells and neighbor not in seen:
                seen.add(neighbor)
                stack.append(neighbor)
    return len(seen) == len(cells)


def check_move(positions, module, step):
    """Mocube で実行できない手なら ValueError."""
    occupied = {c: i for i, c in positions.items()}
    source = positions[module]
    target = add(source, step)
    if module == 0:
        raise ValueError('ID0 は動かせません')
    if sorted(map(abs, step)) != [0, 0, 1]:
        raise ValueError(f'1 軸 1 マスの移動ではありません: {step}')
    if target in occupied:
        raise ValueError(f'ID{module}: 移動先がふさがっています')
    sides = [s for s in SIDES if dot(s, step) == 0]
    if not any(add(source, s) in occupied and add(target, s) in occupied for s in sides):
        raise ValueError(f'ID{module}: 移動の前後で同じ側の支えがありません')
    if not is_connected(c for c in occupied if c != source):
        raise ValueError(f'ID{module}: 動かすと全体が分かれます')


def unreachable_reason(initial, goal):
    """
    1 台ずつの手では届かないことが手を探さずにわかる場合、その理由を返す.

    移動先の支えは移動軸の座標が移動先と同じなので、1 手で外形は広がらない。
    """
    for axis, name in enumerate('xyz'):
        low = min(cell[axis] for cell in initial.values())
        high = max(cell[axis] for cell in initial.values())
        goal_low = min(cell[axis] for cell in goal)
        goal_high = max(cell[axis] for cell in goal)
        if goal_low < low or goal_high > high:
            return (f'{name} 軸の外形が {low}..{high} から {goal_low}..{goal_high} に広がります。'
                    '1 台ずつの手では外形を広げられません')
    return None
