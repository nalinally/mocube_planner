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
| `executor.py` / `plan_executor.py` | 計画を 1 手ずつ頼む (中身 / action クライアントのノード) |
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

`plan_executor` は計画を 1 手ずつ `mocube_executor` の `move_server` に頼む action クライアントです。
1 手の止め方は [mocube_executor の README](../mocube_executor/README.md) にあります。

```bash
ros2 run mocube_bridge device_manager
ros2 run mocube_estimator state_estimator
ros2 run mocube_executor move_server
ros2 run mocube_planner plan_executor --ros-args -p plan_file:=plan.json
```

開始すると **すぐにサーボが回ります**。

1. `move_server` が見え、全モジュールの位置が決まるまで待ちます
2. 実機の形が計画の初期形状と同じか確かめ、計画の ID を実機の ID に座標で読み替えます
3. 1 手を送り、結果を受け取ってから次の手を送ります
4. 手が失敗したら、全モジュールの位置を出して異常終了します

結果が届かないとき (`move_server` が消えた、時間を超えた) は、実行されたか分からないので **送り直しません**。
キャンセルを頼んで今の位置を見て、手の後の形で `move_server` も生きていれば次へ進み、それ以外は異常終了します。

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `plan_file` | | 計画の JSON |
| `position_tolerance_mm` | 10.0 | 形の判定のずれの許容 |
| `start_timeout` | 30.0 | 開始の条件が揃うのを待つ時間 [s] |
| `result_timeout_ratio` / `result_timeout_margin` | 4.0 / 40.0 | 結果を待つ上限 = `duration_s` × ratio + margin [s]。`move_server` の待ち時間より長くします |
| `server_lost_timeout` | 0.5 | `move_server` が見えなくなってから、消えたと判断するまでの時間 [s] |
| `lost_wait` | 1.0 | キャンセルを頼んでから位置を見るまでの時間 [s] |

計画には作ったときの `edge_length_mm` と `speed_gain_mm` が入っていて、`move_server` が自分の設定と照合します。
これらが無い古い計画は読めないので、`plan` で作り直してください。

### 実機なしで試す

`device_manager` の代わりに `sim_modules` を使います。`state_estimator` を計画の初期形状に置きます。

```bash
ros2 run mocube_estimator state_estimator
ros2 run mocube_planner sim_modules --ros-args -p plan_file:=plan.json
ros2 run mocube_executor move_server
ros2 run mocube_planner plan_executor --ros-args -p plan_file:=plan.json
```

実機のモジュールと混ざらないよう、`ROS_DOMAIN_ID` を変えるか、`device_manager` を止めておきます。

| `sim_modules` のパラメータ | 既定値 | 内容 |
|---|---|---|
| `ir_range` | 0.1 | 赤外線を受ける面内のずれ (一辺に対する割合) |
| `ir_only_when_stopped` | false | true なら全サーボの停止中だけ受信します。行き過ぎて戻る動きを試せます |

## 制限

- 1 手で動くのは 1 台だけです。塊ごとの移動は表せません
- 移動先にも同じ軸の座標の支えが必要なので、全体の外形は広がりません。
  `block_to_chair` は外形が広がるので、`plan` は探索せずに「解がありません」と答えます
- 重力は考えていません
- Pathfinder の A* は `Heuristic exhibited non-consistent behavior` で異常終了することがあります。
  これは解がないことの証明ではありません (探し尽くして解がないときは、空の出力で終わります)
- 行き過ぎの判定と戻す量は推定位置 (`speed_gain_mm` の仮の値での積算) を見ています。
  実機で測って合わせるまで、実際の 1.5 マスとは一致しません
- 赤外線が格子点の手前で受かると、その場で止まります。受かる範囲は実機で測っていません
- 2 マス先が空いている手では、行き過ぎた先で接する面のサーボも最初から回します。
  かみ合う相手のいないサーボも回るので、実機で問題がないか確かめる必要があります
