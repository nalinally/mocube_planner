"""初期形状と目標形状から計画 (サーボ指令の列の JSON) を作るコマンド."""

import argparse
import json
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from mocube_planner.backends.pathfinder import PathfinderBackend
from mocube_planner.commands import build_plan, MAX_REPLAY_ERROR, replay
from mocube_planner.model import load_scenario, unreachable_reason

DEFAULT_PATHFINDER = 'external/modular-robotics/build/Pathfinder'


def format_step(number, step):
    servos = ', '.join(
        f"ID{c['module']}.{c['face']}({c['angle_deg']:+.0f})" for c in step['commands'])
    return f"{number:3d}: ID{step['module']} {step['from']} -> {step['to']}  {servos}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('initial', help='初期形状の JSON')
    parser.add_argument('final', help='目標形状の JSON')
    parser.add_argument('-o', '--output', required=True, help='計画の JSON の出力先')
    parser.add_argument('--speed-deg', type=float, default=100.0, help='回転指令の大きさ [deg]')
    parser.add_argument(
        '--pathfinder',
        default=os.environ.get('MOCUBE_PATHFINDER', DEFAULT_PATHFINDER),
        help='Pathfinder の実行ファイル (既定: $MOCUBE_PATHFINDER、なければ WS 直下からの相対パス)')
    parser.add_argument('--moves', help='動きの定義のフォルダ (既定: パッケージの moves/)')
    parser.add_argument('--timeout', type=float, help='探索の制限時間 [s]')
    args = parser.parse_args()

    initial = load_scenario(args.initial)
    goal = set(load_scenario(args.final).values())
    reason = unreachable_reason(initial, goal)
    if reason:
        sys.exit(f'解がありません: {reason}')

    moves_dir = args.moves or Path(get_package_share_directory('mocube_planner')) / 'moves'
    backend = PathfinderBackend(args.pathfinder, moves_dir, timeout=args.timeout)
    plan = build_plan(initial, backend.find_moves(initial, goal), goal, args.speed_deg)
    worst = replay(plan)

    Path(args.output).write_text(
        json.dumps(plan, indent=2, ensure_ascii=False), encoding='utf-8')
    for number, step in enumerate(plan['steps'], 1):
        print(format_step(number, step))
    print(f"{len(plan['steps'])} 手、再生誤差は最大 {worst * 1000:.3f} mm")
    if worst > MAX_REPLAY_ERROR:
        sys.exit('再生誤差が大きすぎます: 指令どおりに動いていません')


if __name__ == '__main__':
    main()
