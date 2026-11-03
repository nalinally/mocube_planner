"""
plan_executor ノード: 計画を 1 手ずつ move_server (action execute_move) に頼んで実行する.

手の結果を受け取ってから次の手を送る。失敗したら全モジュールの位置を出して異常終了する。

結果が届かないとき (サーバが消えた、時間を超えた) は、実行されたか分からないので送り直さない。
キャンセルを頼んだうえで今の位置を見て、手の後の形でサーバも生きていれば次の手へ進み、
それ以外は異常終了する。

コールバックの中で結果を同期的に待つと止まるので、タイマーの状態機械で進める。
"""

from enum import Enum
import json
import sys
import time

from mocube_msgs.action import ExecuteMove
from mocube_msgs.msg import GridPosition, Modules, ModuleStates, ServoCommand
from mocube_planner.executor import assign_ids, before_step, classify_position, load_plan
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

ACTION_NAME = 'execute_move'
CANCEL_FLUSH_S = 0.2


class Phase(Enum):
    WAITING = 'waiting'      # サーバと全モジュールの位置が揃うのを待っている
    SENDING = 'sending'      # ゴールの受付の返事を待っている
    RUNNING = 'running'      # 結果を待っている
    LOST = 'lost'            # 結果が来ないので、キャンセルして様子を見ている


def to_grid_positions(cells):
    return [GridPosition(id=i, x=c[0], y=c[1], z=c[2]) for i, c in sorted(cells.items())]


def to_goal(plan, index):
    """手のゴールを作る."""
    step = plan['steps'][index]
    return ExecuteMove.Goal(
        module=step['module'],
        before=to_grid_positions(before_step(plan, index)),
        after=to_grid_positions(step['positions']),
        commands=[
            ServoCommand(module_id=c['module'], servo=c['servo'], mode=ServoCommand.POSITION,
                         angle=float(c['angle_deg']))
            for c in step['commands']],
        duration_s=float(step['duration_s']),
        edge_length_mm=float(plan['edge_length_mm']),
        speed_gain_mm=float(plan['speed_gain_mm']))


class PlanExecutor(Node):

    def __init__(self):
        super().__init__('plan_executor')

        plan_file = self.declare_parameter('plan_file', '').value
        self._tolerance = self.declare_parameter('position_tolerance_mm', 10.0).value / 1000.0
        self._start_timeout = self.declare_parameter('start_timeout', 30.0).value
        # 手の結果を待つ時間の上限 = duration_s * ratio + margin。move_server の上限より長くする
        self._result_ratio = self.declare_parameter('result_timeout_ratio', 4.0).value
        self._result_margin = self.declare_parameter('result_timeout_margin', 40.0).value
        self._server_lost_timeout = self.declare_parameter('server_lost_timeout', 0.5).value
        self._lost_wait = self.declare_parameter('lost_wait', 1.0).value
        update_period = self.declare_parameter('update_period', 0.1).value

        with open(plan_file, encoding='utf-8') as f:
            self._plan = load_plan(json.load(f))
        self._edge_length = self._plan['edge_length_mm'] / 1000.0

        self.success = False
        self.finished = False
        self._phase = Phase.WAITING
        self._index = 0
        self._deadline = time.monotonic() + self._start_timeout
        self._server_seen = time.monotonic()
        self._goal_handle = None
        self._online = set()
        self._positions = {}

        self._client = ActionClient(self, ExecuteMove, ACTION_NAME)
        self._modules_subscription = self.create_subscription(
            Modules, 'modules', self._on_modules, 10)
        self._states_subscription = self.create_subscription(
            ModuleStates, 'module_states', self._on_module_states, 10)
        self._timer = self.create_timer(update_period, self._on_timer)

    def _on_modules(self, msg):
        self._online = {m.id for m in msg.modules if m.online}

    def _on_module_states(self, msg):
        self._positions = {
            s.id: (s.position.x, s.position.y, s.position.z)
            for s in msg.modules if s.localized}

    def _finish(self, success, message=''):
        self.success = success
        self.finished = True
        if success:
            self.get_logger().info('計画を実行しました')
        else:
            self.get_logger().error(message)

    def _on_timer(self):
        if self.finished:
            return
        if self._phase == Phase.WAITING:
            self._update_waiting()
        elif self._phase in (Phase.SENDING, Phase.RUNNING):
            self._update_running()
        elif self._phase == Phase.LOST:
            self._update_lost()

    def _update_waiting(self):
        count = len(self._plan['initial'])
        located = set(self._positions) & self._online
        if not self._client.server_is_ready() or len(located) < count:
            if time.monotonic() > self._deadline:
                self._finish(False, (
                    f'{count} 台と move_server が揃いません: '
                    f'サーバ {self._client.server_is_ready()}, 揃った ID {sorted(located)}'))
            return
        try:
            self._plan = assign_ids(
                self._plan, self._positions, self._edge_length, self._tolerance)
        except ValueError as e:
            self._finish(False, str(e))
            return
        self._send_step()

    def _send_step(self):
        if self._index >= len(self._plan['steps']):
            self._finish(True)
            return
        step = self._plan['steps'][self._index]
        self.get_logger().info(
            f"{self._index + 1}/{len(self._plan['steps'])}: "
            f"ID{step['module']} {tuple(step['from'])} -> {tuple(step['to'])}")
        self._phase = Phase.SENDING
        self._goal_handle = None
        self._server_seen = time.monotonic()
        self._deadline = (time.monotonic() + step['duration_s'] * self._result_ratio
                          + self._result_margin)
        future = self._client.send_goal_async(to_goal(self._plan, self._index))
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self._fail_step('move_server に断られました (別の手を実行中)')
            return
        self._goal_handle = handle
        self._phase = Phase.RUNNING
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if self._phase == Phase.LOST:
            return  # 位置で判断する途中に遅れて届いた結果は使わない
        if result.outcome != ExecuteMove.Result.SUCCEEDED:
            self._fail_step(f'{result.message} (outcome={result.outcome})', result.final_states)
            return
        self._index += 1
        self._send_step()

    def _fail_step(self, message, states=()):
        where = f"{self._index + 1}/{len(self._plan['steps'])} 手目"
        cells = {s.id: (round(s.position.x, 4), round(s.position.y, 4), round(s.position.z, 4))
                 for s in states if s.localized}
        self._finish(False, f'{where}: {message}. 終了時の位置 [m]: {cells}')

    def _update_running(self):
        now = time.monotonic()
        if self._client.server_is_ready():
            self._server_seen = now
        reason = ''
        if now - self._server_seen > self._server_lost_timeout:
            reason = 'move_server が見えなくなりました'
        elif now > self._deadline:
            reason = '結果が時間内に届きません'
        if reason:
            self.get_logger().error(f'{self._index + 1} 手目: {reason}。送り直さずに様子を見ます')
            self.cancel()
            self._phase = Phase.LOST
            self._deadline = now + self._lost_wait

    def _update_lost(self):
        if time.monotonic() < self._deadline:
            return
        state = classify_position(
            self._positions, before_step(self._plan, self._index),
            self._plan['steps'][self._index]['positions'], self._edge_length, self._tolerance)
        if state == 'after' and self._client.server_is_ready():
            self.get_logger().warning(f'{self._index + 1} 手目は実行済みと判断して進みます')
            self._index += 1
            self._send_step()
            return
        names = {'before': '手の前', 'after': '手の後', 'unknown': 'どちらでもない'}
        self._finish(False, (
            f'{self._index + 1} 手目の結果が分かりません: 今の形は{names[state]}です。'
            f'サーバ {self._client.server_is_ready()}, 位置 [m] {self._positions}'))

    def cancel(self):
        """実行中の手のキャンセルを頼む. 届く保証はない (move_server は相手が消えても止まる)."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)

    plan_executor = PlanExecutor()

    try:
        while rclpy.ok() and not plan_executor.finished:
            rclpy.spin_once(plan_executor, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        plan_executor.cancel()
        try:
            end = time.monotonic() + CANCEL_FLUSH_S
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(plan_executor, timeout_sec=0.02)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        success = plan_executor.success
        plan_executor.destroy_node()
        rclpy.try_shutdown()
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
