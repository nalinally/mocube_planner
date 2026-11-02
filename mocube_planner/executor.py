"""
計画の実行の中身: 1 手ごとにサーボを回し、止めて、位置と赤外線を確かめて次へ進む.

ROS に依存しないので単体テストできる。ノードは plan_executor.py。

モジュールは見分けがつかないので、計画の ID は開始時に実機の ID へ座標で読み替える。
ID0 は計画でも実機でも原点。
"""

from dataclasses import dataclass, field
from enum import Enum

from mocube_estimator.estimator import SERVO_MAX_DEG
from mocube_estimator.geometry import FACES, POSITIVE_FACE
from mocube_planner.model import contacts
import numpy as np

STOP_ANGLE = 0.0  # 回転モードでは 0 で停止


class Status(Enum):
    WAITING = 'waiting'    # 全モジュールの位置が揃うのを待っている
    DRIVING = 'driving'    # サーボを回している
    VERIFYING = 'verifying'  # 止めて、位置と赤外線の確認を待っている
    DONE = 'done'
    FAILED = 'failed'


@dataclass
class Observation:
    """ある時刻のモジュールの様子."""

    online: set = field(default_factory=set)
    positions: dict = field(default_factory=dict)  # ID -> 位置 [m] (位置が決まったものだけ)
    receptions: dict = field(default_factory=dict)  # (ID, 面) -> (送信元 ID, 受信時刻)


def load_plan(data):
    """JSON を読んだ dict の ID と座標を整数に揃える."""
    def cells(positions):
        return {int(i): tuple(c) for i, c in positions.items()}

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
    return {'initial': cells(data['initial']), 'steps': steps}


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
    return {'initial': {table[i]: c for i, c in plan['initial'].items()}, 'steps': steps}


def expected_receptions(before, after):
    """手で新しくできる接触について、赤外線を受けるはずの (受信側 ID, 面, 送信元 ID) を返す."""
    return {
        (lower, FACES[POSITIVE_FACE[axis]], upper)
        for lower, upper, axis in contacts(after) - contacts(before)
    }


class PlanRunner:
    """
    計画を 1 手ずつ進める.

    update() を周期的に呼ぶと、今送るべきサーボ指令 [(ID, サーボ, 回転指令), ...] を返す。
    時刻は赤外線の受信時刻と同じ UNIX time [s]。
    """

    def __init__(self, plan, edge_length, position_tolerance, verify_timeout,
                 start_timeout, require_ir=True):
        """
        パラメータ.

        plan: load_plan() の計画
        edge_length: モジュールの一辺 [m]
        position_tolerance: 手の後の位置のずれの許容 [m]
        verify_timeout: 手の後、確認が取れるまで待つ時間 [s]
        start_timeout: 開始時に全モジュールの位置が揃うまで待つ時間 [s]
        require_ir: 新しくできる接触の赤外線の受信も確認するか
        """
        self.plan = plan
        self.edge_length = edge_length
        self.position_tolerance = position_tolerance
        self.verify_timeout = verify_timeout
        self.start_timeout = start_timeout
        self.require_ir = require_ir
        self.status = Status.WAITING
        self.error = ''
        self.index = 0  # 実行中の手の番号
        self._deadline = None
        self._active = []  # 回している (ID, サーボ)
        self._expected = set()
        self._step_start = 0.0

    @property
    def finished(self):
        return self.status in (Status.DONE, Status.FAILED)

    def update(self, now, observation):
        if self.status == Status.WAITING:
            return self._update_waiting(now, observation)
        if self.status == Status.DRIVING:
            return self._update_driving(now, observation)
        if self.status == Status.VERIFYING:
            return self._update_verifying(now, observation)
        return []

    def stop(self):
        """回しているサーボを止める指令を返す (中断用)."""
        commands = [(module, servo, STOP_ANGLE) for module, servo in self._active]
        self._active = []
        return commands

    def _fail(self, message):
        self.status = Status.FAILED
        self.error = message
        return self.stop()

    def _update_waiting(self, now, observation):
        if self._deadline is None:
            self._deadline = now + self.start_timeout
        ready = (len(observation.positions) >= len(self.plan['initial'])
                 and set(observation.positions) <= observation.online)
        if not ready:
            if now > self._deadline:
                return self._fail(
                    f"{len(self.plan['initial'])} 台が揃いません: "
                    f'位置あり {sorted(observation.positions)}, '
                    f'オンライン {sorted(observation.online)}')
            return []
        try:
            self.plan = assign_ids(
                self.plan, observation.positions, self.edge_length, self.position_tolerance)
        except ValueError as e:
            return self._fail(str(e))
        return self._start_step(now)

    def _start_step(self, now):
        if self.index >= len(self.plan['steps']):
            self.status = Status.DONE
            return []
        step = self.plan['steps'][self.index]
        before = (self.plan['steps'][self.index - 1]['positions']
                  if self.index else self.plan['initial'])
        self._expected = expected_receptions(before, step['positions'])
        self._step_start = now
        self._deadline = now + step['duration_s']
        self._active = [(c['module'], c['servo']) for c in step['commands']]
        self.status = Status.DRIVING
        return [(c['module'], c['servo'], c['angle_deg']) for c in step['commands']]

    def _update_driving(self, now, observation):
        missing = {module for module, _ in self._active} - observation.online
        if missing:
            return self._fail(f'{self.index + 1} 手目: オフラインになりました: {sorted(missing)}')
        if now < self._deadline:
            return []
        self.status = Status.VERIFYING
        self._deadline = now + self.verify_timeout
        return self.stop()

    def _position_errors(self, observation):
        wrong = {}
        for module, cell in self.plan['steps'][self.index]['positions'].items():
            position = observation.positions.get(module)
            if position is None:
                wrong[module] = None
                continue
            error = np.linalg.norm(np.asarray(position) - np.array(cell) * self.edge_length)
            if error > self.position_tolerance:
                wrong[module] = float(error)
        return wrong

    def _missing_receptions(self, observation):
        if not self.require_ir:
            return set()
        missing = set()
        for module, face, sender in self._expected:
            reception = observation.receptions.get((module, face))
            if reception is None or reception[0] != sender or reception[1] < self._step_start:
                missing.add((module, face, sender))
        return missing

    def _update_verifying(self, now, observation):
        wrong = self._position_errors(observation)
        missing = self._missing_receptions(observation)
        if not wrong and not missing:
            self.index += 1
            return self._start_step(now)
        if now > self._deadline:
            return self._fail(
                f'{self.index + 1} 手目の確認が取れません: 位置のずれ [m] {wrong}, '
                f'受信できていない赤外線 (受信側, 面, 送信元) {sorted(missing)}')
        return []
