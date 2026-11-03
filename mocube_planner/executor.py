"""
計画を 1 手ずつ実行するための、ROS に依存しない部分. ノードは plan_executor.py.

モジュールは見分けがつかないので、計画の ID は開始時に実機の ID へ座標で読み替える。
ID0 は計画でも実機でも原点。手そのものの実行は mocube_executor の move_server が受け持つ。
"""

from mocube_estimator.estimator import SERVO_MAX_DEG
import numpy as np


def load_plan(data):
    """JSON を読んだ dict の ID と座標を整数に揃える. 古い形式や範囲外の指令なら ValueError."""
    def cells(positions):
        return {int(i): tuple(c) for i, c in positions.items()}

    if 'edge_length_mm' not in data or 'speed_gain_mm' not in data:
        raise ValueError('計画に edge_length_mm と speed_gain_mm がありません。plan で作り直してください')
    steps = []
    for step in data['steps']:
        for command in step['commands']:
            if abs(command['angle_deg']) > SERVO_MAX_DEG:
                raise ValueError(f"回転指令が範囲外です: {command['angle_deg']} deg")
        steps.append({
            **step,
            'positions': cells(step['positions']),
            'commands': [dict(c) for c in step['commands']],
        })
    return {
        'edge_length_mm': data['edge_length_mm'],
        'speed_gain_mm': data['speed_gain_mm'],
        'initial': cells(data['initial']),
        'steps': steps,
    }


def assign_ids(plan, positions, edge_length, tolerance):
    """
    計画の ID を実機の ID に読み替えた計画を返す. 形が計画の初期形状と違えば ValueError.

    positions: 実機の ID -> 位置 [m]
    """
    actual = {}
    for module, position in positions.items():
        cell = tuple(int(v) for v in np.rint(np.asarray(position) / edge_length))
        error = np.linalg.norm(np.asarray(position) - np.array(cell) * edge_length)
        if error > tolerance:
            raise ValueError(f'ID{module} が格子点にいません: {tuple(position)}')
        if cell in actual:
            raise ValueError(f'ID{module} と ID{actual[cell]} が同じ座標 {cell} にいます')
        actual[cell] = module
    expected = set(plan['initial'].values())
    if set(actual) != expected:
        raise ValueError(
            '実機の形が計画の初期形状と違います: '
            f'足りない座標 {sorted(expected - set(actual))}, '
            f'余分な座標 {sorted(set(actual) - expected)}')

    table = {module: actual[cell] for module, cell in plan['initial'].items()}
    steps = [{
        **step,
        'module': table[step['module']],
        'commands': [{**c, 'module': table[c['module']]} for c in step['commands']],
        'positions': {table[i]: cell for i, cell in step['positions'].items()},
    } for step in plan['steps']]
    return {**plan, 'initial': {table[i]: c for i, c in plan['initial'].items()}, 'steps': steps}


def before_step(plan, index):
    """手の前の {ID: 格子座標} を返す."""
    return plan['steps'][index - 1]['positions'] if index else plan['initial']


def classify_position(positions, before, after, edge_length, tolerance):
    """
    結果を受け取れなかった手が実行されたかを、今の位置から判断する.

    positions: ID -> 位置 [m]。戻り値は 'before' / 'after' / 'unknown'。
    """
    def matches(cells):
        for module, cell in cells.items():
            position = positions.get(module)
            if position is None:
                return False
            error = np.linalg.norm(np.asarray(position) - np.array(cell) * edge_length)
            if error > tolerance:
                return False
        return True

    if matches(after):
        return 'after'
    if matches(before):
        return 'before'
    return 'unknown'
