"""
plan_executor ノード.

plan コマンドが作った計画を読み、/servo_commands に指令を出して実行する。
/module_states と /modules で 1 手ごとに確かめ、ずれたら止めて異常終了する。
DDS の検出前に出した指令は失われるので、指令の購読者が揃うまで始めない。
"""

import json
import sys
import time

from mocube_msgs.msg import Modules, ModuleStates, ServoCommand, ServoCommands
from mocube_planner.executor import load_plan, Observation, PlanRunner, Status
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PlanExecutor(Node):

    def __init__(self):
        super().__init__('plan_executor')

        plan_file = self.declare_parameter('plan_file', '').value
        edge_length_mm = self.declare_parameter('edge_length_mm', 113.1).value
        tolerance_mm = self.declare_parameter('position_tolerance_mm', 10.0).value
        verify_timeout = self.declare_parameter('verify_timeout', 3.0).value
        start_timeout = self.declare_parameter('start_timeout', 30.0).value
        require_ir = self.declare_parameter('require_ir', True).value
        update_period = self.declare_parameter('update_period', 0.02).value
        # device_manager と state_estimator
        self._min_subscribers = self.declare_parameter('min_servo_subscribers', 2).value

        with open(plan_file, encoding='utf-8') as f:
            plan = load_plan(json.load(f))
        self._runner = PlanRunner(
            plan, edge_length_mm / 1000.0, tolerance_mm / 1000.0,
            verify_timeout, start_timeout, require_ir)
        self._observation = Observation()
        self._reported_index = -1
        self.success = False

        self._publisher = self.create_publisher(ServoCommands, 'servo_commands', 10)
        self._modules_subscription = self.create_subscription(
            Modules, 'modules', self._on_modules, 10)
        self._states_subscription = self.create_subscription(
            ModuleStates, 'module_states', self._on_module_states, 10)
        self._timer = self.create_timer(update_period, self._on_timer)

    def _on_modules(self, msg):
        self._observation.online = {m.id for m in msg.modules if m.online}
        self._observation.receptions = {
            (m.id, r.face): (r.address, r.t) for m in msg.modules for r in m.receivers}

    def _on_module_states(self, msg):
        self._observation.positions = {
            s.id: (s.position.x, s.position.y, s.position.z)
            for s in msg.modules if s.localized}

    def _on_timer(self):
        ready = self._publisher.get_subscription_count() >= self._min_subscribers
        observation = self._observation if ready else Observation()
        self.send(self._runner.update(time.time(), observation))
        runner = self._runner
        if runner.status == Status.DRIVING and runner.index != self._reported_index:
            self._reported_index = runner.index
            step = runner.plan['steps'][runner.index]
            self.get_logger().info(
                f"{runner.index + 1}/{len(runner.plan['steps'])}: "
                f"ID{step['module']} {tuple(step['from'])} -> {tuple(step['to'])}")
        if runner.finished:
            self.success = runner.status == Status.DONE
            if self.success:
                self.get_logger().info('計画を実行しました')
            else:
                self.get_logger().error(runner.error)
            raise SystemExit

    def stop(self):
        self.send(self._runner.stop())

    def send(self, commands):
        if not commands:
            return
        msg = ServoCommands()
        for module, servo, angle in commands:
            msg.commands.append(ServoCommand(
                module_id=module, servo=servo, mode=ServoCommand.POSITION, angle=float(angle)))
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    plan_executor = PlanExecutor()

    try:
        rclpy.spin(plan_executor)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        # 中断でもサーボを回したままにしない
        plan_executor.stop()
        success = plan_executor.success
        plan_executor.destroy_node()
        rclpy.try_shutdown()
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
