"""
sim_modules ノード: 実機なしで計画を試すための、device_manager の代わり.

計画の全モジュールをオンラインとして /modules に流し、state_estimator を計画の初期形状に置く。
隣接する組の赤外線の受信は、全サーボが止まっている間だけ /module_states の位置から作る。
"""

import json
import time

from mocube_estimator.geometry import FACES, POSITIVE_FACE
from mocube_msgs.msg import (
    GridPosition,
    IrReceiver,
    Module,
    Modules,
    ModuleStates,
    ServoCommands,
)
from mocube_msgs.srv import SetGridPositions
from mocube_planner.executor import load_plan
from mocube_planner.model import contacts
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

RECEIVER_FACES = ('u', 'r', 'f')
ON_GRID_RATIO = 0.1  # 一辺に対する割合


class SimModules(Node):

    def __init__(self):
        super().__init__('sim_modules')

        plan_file = self.declare_parameter('plan_file', '').value
        self._edge_length = self.declare_parameter('edge_length_mm', 113.1).value / 1000.0
        publish_period = self.declare_parameter('publish_period', 0.1).value

        with open(plan_file, encoding='utf-8') as f:
            self._initial = load_plan(json.load(f))['initial']
        self._positions = {}
        self._receptions = {}

        self._publisher = self.create_publisher(Modules, 'modules', 10)
        self._subscription = self.create_subscription(
            ModuleStates, 'module_states', self._on_module_states, 10)
        # 動きは state_estimator が積算するので、ここでは回っているサーボだけ覚える
        self._driving = set()
        self._servo_subscription = self.create_subscription(
            ServoCommands, 'servo_commands', self._on_servo_commands, 10)
        self._client = self.create_client(SetGridPositions, 'set_grid_positions')
        self._timer = self.create_timer(publish_period, self._on_timer)

    def _on_module_states(self, msg):
        self._positions = {
            s.id: np.array([s.position.x, s.position.y, s.position.z])
            for s in msg.modules if s.localized}

    def _on_servo_commands(self, msg):
        for command in msg.commands:
            if command.angle != 0.0:
                self._driving.add((command.module_id, command.servo))
            else:
                self._driving.discard((command.module_id, command.servo))

    def _on_timer(self):
        if set(self._positions) != set(self._initial) and self._client.service_is_ready():
            request = SetGridPositions.Request()
            for module, cell in self._initial.items():
                request.positions.append(
                    GridPosition(id=module, x=cell[0], y=cell[1], z=cell[2]))
            self._client.call_async(request)

        self._update_receptions(time.time())
        msg = Modules()
        for module in sorted(self._initial):
            state = Module(id=module, online=True, last_seen=time.time())
            for face in RECEIVER_FACES:
                address, t = self._receptions.get((module, face), (-1, 0.0))
                state.receivers.append(IrReceiver(face=face, address=address, t=t))
            msg.modules.append(state)
        self._publisher.publish(msg)

    def _update_receptions(self, now):
        if self._driving:
            return
        cells = {}
        for module, position in self._positions.items():
            cell = np.rint(position / self._edge_length)
            if np.linalg.norm(position / self._edge_length - cell) <= ON_GRID_RATIO:
                cells[module] = tuple(int(v) for v in cell)
        for lower, upper, axis in contacts(cells):
            self._receptions[(lower, FACES[POSITIVE_FACE[axis]])] = (upper, now)


def main(args=None):
    rclpy.init(args=args)

    sim_modules = SimModules()

    try:
        rclpy.spin(sim_modules)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        sim_modules.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
