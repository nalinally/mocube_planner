# mocube_planner

mocube の変形計画と、その実行を受け持つ ROS 2 パッケージです。
初期形状と目標形状から手の列を探し、サーボ指令の列 (計画) にして、確かめながら実行します。

```
初期形状と目標形状 -> 探索器 -> 手の列 -> 検査 -> サーボ指令 -> estimator で再生確認 -> plan_executor
```

| ファイル | 役割 |
|---|---|
| `model.py` | 状態 (ID -> 格子座標)、手の検査、シナリオの読み込み |
| `backends/pathfinder.py` | 探索器。Pathfinder を外部プロセスとして呼ぶ |
| `commands.py` | 手 -> サーボ指令、`mocube_estimator` の積算での再生確認 |
| `executor.py` / `plan_executor.py` | 計画の実行 (中身 / ROS ノード) |
| `sim_modules.py` | 実機なしで試すための `device_manager` の代わり |
| `render_plan.py` | 計画を GIF にする |

探索器は `find_moves(initial, goal)` を持つクラスで、`{ID: 座標}` と目標の座標の集合から
手の列 `[(ID, 移動量), ...]` を返します。別の探索器は `backends/` に足します。

## 計画を作る

Pathfinder は WS の `external/modular-robotics` にあり、`external/build_pathfinder.sh` でビルドします。
WS 直下で実行するか、`--pathfinder` または環境変数 `MOCUBE_PATHFINDER` で実行ファイルを指定します。

```bash
S=src/mocube_planner/scenarios
ros2 run mocube_planner plan $S/l_wall_to_floor_initial.json $S/l_wall_to_floor_final.json -o plan.json
ros2 run mocube_planner render_plan plan.json -o plan.gif
```

各手が mocube の動きとして実行できるか、目標形状に着くかを検査し、積算で再生して確かめます
(誤差が 1 mm を超えると異常終了)。

シナリオの JSON は `static` のモジュールが ID0 で、その座標が原点です。

## 動きの定義 (`moves/Mocube.json`)

```
?!
##
```

`?` が動くモジュール、`!` が移動先、`#` が必要な支えです。`permGen` で全方向に展開されます。

- 支えは移動の前後とも同じ側に必要です。角の回り込みはできません
- 移動の向きの前後に接触があってもかまいません。面に垂直な向きはサーボで動かせませんが、拘束もされません

## 実行する

```bash
ros2 run mocube_bridge device_manager
ros2 run mocube_estimator state_estimator
ros2 run mocube_planner plan_executor --ros-args -p plan_file:=plan.json
```

開始すると **すぐにサーボが回ります**。

1. `/servo_commands` の購読者が揃い、全モジュールの位置が決まるまで待ちます
2. 実機の形が計画の初期形状と同じか確かめ、計画の ID を実機の ID に座標で読み替えます
3. 1 手ごとにサーボを回して止め、位置のずれと、新しくできた接触の赤外線の受信を確かめます
4. 確認が取れない、モジュールがオフラインになる、のどれかでサーボを止めて異常終了します

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `plan_file` | | 計画の JSON |
| `edge_length_mm` | 113.1 | モジュールの一辺 |
| `position_tolerance_mm` | 10.0 | 手の後の位置のずれの許容 |
| `verify_timeout` | 3.0 | 手の後、確認を待つ時間 [s] |
| `start_timeout` | 30.0 | 開始の条件が揃うのを待つ時間 [s] |
| `require_ir` | true | 赤外線の受信も確かめるか |
| `min_servo_subscribers` | 2 | 開始に必要な `/servo_commands` の購読者数 |

### 実機なしで試す

`device_manager` の代わりに `sim_modules` を使います。`state_estimator` を計画の初期形状に置きます。

```bash
ros2 run mocube_estimator state_estimator
ros2 run mocube_planner sim_modules --ros-args -p plan_file:=plan.json
ros2 run mocube_planner plan_executor --ros-args -p plan_file:=plan.json
```

実機のモジュールと混ざらないよう、`ROS_DOMAIN_ID` を変えるか、`device_manager` を止めておきます。

## 制限

- 1 手で動くのは 1 台だけです。塊ごとの移動は表せません
- 移動先にも同じ軸の座標の支えが必要なので、全体の外形は広がりません。
  `block_to_chair` は外形が広がるので、`plan` は探索せずに「解がありません」と答えます
- 重力は考えていません
- Pathfinder の A* は `Heuristic exhibited non-consistent behavior` で異常終了することがあります。
  これは解がないことの証明ではありません (探し尽くして解がないときは、空の出力で終わります)
- 手の時間は `speed_gain_mm` の仮の値から計算しています。実機で測って合わせる必要があります
- `state_estimator` は赤外線を受けるたびに位置を 1 マス隣にリセットします。
  滑っている途中でも受信が続くと、推定位置が引き戻されるかもしれません (実機未確認)。
  途中でも受信を作っていた版の `sim_modules` で、半マス足りずに止まったことが 1 度あります (原因は未特定)
