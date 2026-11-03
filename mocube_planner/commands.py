"""
手の列をサーボ指令の列 (計画) に変換し、mocube_estimator の積算で再生して確かめる.

動くモジュールが接触しているすべての面で、同じ相対移動になるようサーボを回す。
赤外線を受けずに 1.5 マスまで行き過ぎたときに、止まったピニオンにかみ込まないよう、
2 マス先が空いていれば、そこで接する面のサーボも回しておく。
"""

from mocube_estimator.estimator import Estimator
from mocube_estimator.geometry import DRIVES, FACES, NEGATIVE_FACE, POSITIVE_FACE
from mocube_planner.model import add, check_move, dot, SIDES
import numpy as np

EDGE_LENGTH_MM = 113.1  # state_estimator と move_server の edge_length_mm
SPEED_GAIN_MM = 0.2     # state_estimator と move_server の speed_gain_mm [mm/s/deg]
EDGE_LENGTH = EDGE_LENGTH_MM / 1000.0
SPEED_GAIN = SPEED_GAIN_MM / 1000.0
MAX_REPLAY_ERROR = 0.001  # [m]


def servo_commands(positions, module, step, speed_deg):
    """{(ID, サーボ): 回転指令 [deg]} を返す. 移動の前後と行き過ぎた先で接触する面をすべて回す."""
    occupied = {c: i for i, c in positions.items()}
    source = positions[module]
    commands = {}
    destination = add(source, step)
    beyond = add(destination, step)
    # 2 マス先にモジュールがいるとき、隣の面のサーボはそのモジュールを押してしまう
    cells = (source, destination) if beyond in occupied else (source, destination, beyond)
    for side in SIDES:
        if dot(side, step) != 0:
            continue
        axis = int(np.flatnonzero(side)[0])
        for cell in cells:
            other = occupied.get(add(cell, side))
            if other is None:
                continue
            lower, upper = (module, other) if side[axis] > 0 else (other, module)
            # lower に対する upper の速度の向き
            relative = -np.array(step) if side[axis] > 0 else np.array(step)
            lower_face, upper_face = POSITIVE_FACE[axis], NEGATIVE_FACE[axis]
            if abs(np.dot(DRIVES[lower_face], relative)) > 0.5:
                key = (lower, lower_face)
                angle = speed_deg * np.dot(DRIVES[lower_face], relative)
            else:
                # upper のサーボは lower を動かすので符号が逆になる
                key = (upper, upper_face)
                angle = -speed_deg * np.dot(DRIVES[upper_face], relative)
            if commands.setdefault(key, float(angle)) != float(angle):
                raise ValueError(f'サーボ {key} への指令が食い違っています')
    return commands


def build_plan(initial, moves, goal, speed_deg=100.0):
    """
    手の列 [(ID, 移動量), ...] を検査して計画 (JSON にできる dict) にする.

    実行できない手が混ざっている、または目標形状に着かないなら ValueError。
    """
    positions = dict(initial)
    duration = EDGE_LENGTH / (SPEED_GAIN * speed_deg)
    steps = []
    for module, step in moves:
        check_move(positions, module, step)
        commands = servo_commands(positions, module, step, speed_deg)
        source = positions[module]
        positions = {**positions, module: add(source, step)}
        steps.append({
            'module': module,
            'from': source,
            'to': positions[module],
            'duration_s': duration,
            'commands': [
                {'module': m, 'servo': s, 'face': FACES[s], 'angle_deg': a}
                for (m, s), a in sorted(commands.items())
            ],
            'positions': dict(positions),
        })
    if set(positions.values()) != set(goal):
        raise ValueError('手の列が目標形状に着きません')
    return {
        'edge_length_mm': EDGE_LENGTH_MM,
        'speed_gain_mm': SPEED_GAIN_MM,
        'initial': dict(initial),
        'steps': steps,
    }


def replay(plan):
    """計画の指令を積算し、各手の後の格子からのずれの最大値 [m] を返す."""
    estimator = Estimator(EDGE_LENGTH, SPEED_GAIN)
    estimator.update_modules([(i, True) for i in plan['initial']])
    estimator.set_grid_positions(plan['initial'], 0.0)
    worst = 0.0
    for step in plan['steps']:
        for command in step['commands']:
            estimator.set_servo(command['module'], command['servo'], command['angle_deg'])
        ticks = 1000
        for _ in range(ticks):
            estimator.step(step['duration_s'] / ticks)
        for command in step['commands']:
            estimator.set_servo(command['module'], command['servo'], None)
        for i, cell in step['positions'].items():
            error = np.linalg.norm(estimator.modules[i].position - np.array(cell) * EDGE_LENGTH)
            worst = max(worst, float(error))
    return worst
