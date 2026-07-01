# fc 安全しきい値の較正実験

DRAGON の **可制御レンチ内接半径** `fc_f_min` / `fc_t_min` に対する
Dracomancer 安全ゲートのしきい値 `[hard_min, min]`（[volume_radius_monitor.py](../scripts/safety/volume_radius_monitor.py)）を、
**データに基づいて**決めるための実験手順と結果を記録する。

関連: [README.md](../README.md)（安全スケールの定義）。

## 1. 背景：fc とは何か、外乱はどう関わるか

`fc_f_min` / `fc_t_min` は [robot_model.cpp](../../../aerial_robot_model/src/model/base_model/robot_model.cpp#L495-L603) で計算される、
**可制御力／トルク凸包の内接半径**である。ロータ配置とスラスト制約のみで決まり、
**機体形状（内部関節角）の関数**である。

```text
fc_f_min[N]    = その姿勢で「あらゆる方向に保証して出せる力」の最大値
              = 最悪方向の外乱力を打ち消せる限界
fc_t_min[N·m]  = 同上（トルク）
```

重要な帰結：**外乱を加えても fc 値そのものは変化しない**。
したがって実験は外乱で fc を変えるのではなく、次の 2 部構成とする。

| Part | 目的 | 外乱の役割 |
| --- | --- | --- |
| **A. 形状スイープ** | 関節角空間での fc 分布・「崖」を把握 | なし（形状のみ変える） |
| **B. 外乱検証** | 「fc = 外乱除去能力」を実証し `hard_min` の安全係数を裏付ける | fc の妥当性検証 |

## 2. 実験環境

- DRAGON quad（`full_vectoring` 制御）、Gazebo シミュレーション、ホバリング（`flight_state == 5`）。
- 起動: `roslaunch dragon bringup.launch sim:=true headless:=true`
- アーム＋離陸: `/dragon/teleop_command/start` → `/dragon/teleop_command/takeoff`
- 計測対象トピック:
  - `/dragon/debug/fc_f_min`, `/dragon/debug/fc_t_min`（`std_msgs/Float64`、40 Hz）
  - `/dragon/joint_states`, `/dragon/joints_ctrl`, `/dragon/uav/cog/odom`, `/dragon/flight_state`
- 形状指令: `/dragon/joints_ctrl`（6関節 `joint{1,2,3}_{pitch,yaw}`）
- 外乱注入: `/gazebo/apply_body_wrench`（`dragon::root` に step wrench）

ツール（すべて [scripts/experiments/](../scripts/experiments/)）:

| ファイル | 役割 |
| --- | --- |
| `collect_fc_data.py` | Part A。形状を格子・ランダムに振り、整定後の fc を CSV 記録。低高度で自動アボート |
| `disturbance_test.py` | Part B。固定形状で外力/トルクを増加させ、COG誤差から破綻点を計測 |
| `analyze_fc_data.py` | CSV から fc 分布を集計し `[hard_min, min]` を提案（オフライン・ROS不要） |
| `run_fc_experiment.sh` | rosbag→Part A→(Part B)→解析の一括実行 |

## 3. Part A：形状スイープ手順

1. ホバリング状態でノミナル形状 `[0, π/2, 0, π/2, 0, π/2]`（離陸時の円形姿勢）から開始。
2. **1次元スイープ**：6関節を 1 つずつ可動範囲で振る（pitch ∈ [-0.6, 0.6]、yaw ∈ [0, π/2]、各7点）。他はノミナル。
3. **ランダムサンプリング**：可動域内で 6 関節同時にランダム設定（40点）。
4. 各形状は `max_step=0.05 rad/cmd` で緩やかに移動し、`settle_time=2.5 s` 整定後、`sample_time=1.0 s` 間の fc を平均して 1 行記録。
5. 高度が `abort_altitude=0.4 m` を下回ったら（制御破綻）ノミナルへ戻して終了。

記録列: `label, fc_f_min_mean/std, fc_t_min_mean/std, altitude_mean, cmd_<joint>, meas_<joint>`。

データ取得は rosbag にも保存（`fc_sweep.bag`）。再生で任意トピックを再解析可能。

## 4. Part B：外乱検証手順

1. 代表形状（高 fc／低 fc）でホバリング。
2. `/gazebo/apply_body_wrench` で `dragon::root` に step wrench（軸ごと：fx, fz, tx …）を `duration=2 s` 印加。
3. 大きさを段階的に増加（例 0.5→5.0）。COG 位置/姿勢誤差の最大値を記録。
4. 誤差が `error_limit` を超えたら「制御喪失」とし、その直前の大きさを**実効的な外乱除去能力**とする。
5. 各形状の fc と突き合わせ、`fc ≈ 除去可能外乱` の関係と安全係数を評価。

## 5. しきい値の決め方（解析方針）

[analyze_fc_data.py](../scripts/experiments/analyze_fc_data.py) で fc 分布から提案する。

```text
hard_min = max(0, percentile(fc, hard_pct) × safety)   # 危険下限＝崖の手前
min      = median(fc) × min_frac                        # 完全安全点＝典型値の一定割合
```

- `hard_min`：到達しうる形状での fc 下側分位（既定 p5、崖の直上）。Part B の安全係数を掛ける。
- `min`：典型（中央値）fc の `min_frac`（既定 0.8）。これ以上で安全スケール=1.0。
- **力・トルクは桁が違うため独立に決める**。

## 6. 結果（このリポジトリでの実測）

> 取得日: 2026-06-30 / DRAGON quad sim / ホバ高度 1.0 m
> 生成物: `/tmp/dracomancer_fc_experiment_20260630_001/`
> 可視化: [../figures/fc_thresholds_83samples/](../figures/fc_thresholds_83samples/)

<!-- RESULTS:BEGIN -->
### Part A：fc 分布（83 形状サンプル）

| 量 | min | p5 | p25 | median | p75 | max |
| --- | --- | --- | --- | --- | --- | --- |
| `fc_f_min` | 0.0346 | 0.0705 | 0.1077 | **0.1709** | 0.2785 | 1.5167 |
| `fc_t_min` | 0.0000 | 0.0003 | 0.0065 | **0.0735** | 0.2678 | 1.8917 |

- ノミナル円形姿勢（離陸形状）は今回の初期サンプルで `fc_f≈0.160 / fc_t≈0.0008`。特にトルクはほぼ 0 で、円形姿勢はトルク可制御性が乏しい。
- 全サンプルで**ホバリングは破綻せず**（高度1.0m維持）。純ホバではどの形状も保持可能。

### Part B：外乱除去（ノミナル形状・補助検証）

| 軸 | 最大入力 | 最大COG誤差[m] | 制御保持 |
| --- | --- | --- | --- |
| `fx` | 5 N | 0.177 | yes |
| `fz` | 5 N | 0.274 | yes |
| `tx` | 2 N | 0.061 | yes |
| `tx` | 3 N | 1.133 | no |

- 外乱検証中の fc は未受信タイミングで 0 と記録された行があるため、ここではしきい値決定の主データではなく補助検証として扱う。
- **重要**：fc は絶対 N / N·m の限界値として直接扱わず、**fc 分布の相対位置**でしきい値を決める。

### 現在適用しているしきい値（4096グリッドのデータ駆動）

解析スクリプトの標準規則に従い、`hard_min = p5`、`min = median × 0.8` とした。
この規則は制御分野の標準値ではなく、観測された形状分布上で低fc領域を避けるための経験的な初期値である。
4096グリッドの広い形状分布を反映した現在の実装値は以下。

| 軸 | hard_min | min | 根拠 |
| --- | --- | --- | --- |
| force | **0.108990** | **0.249220** | hard=p5、min=median×0.8 |
| torque | **0.015400** | **0.278159** | hard=p5、min=median×0.8 |

- 83サンプル値 `force[0.070544, 0.136743] / torque[0.000345, 0.058762]` から、4096グリッド再取得結果に基づく値へ更新した。
- 4096グリッド中の高度低下ケースは `fc_*_min` だけで分離できないため、高度低下・墜落回避は別途、遷移中の高度、鉛直速度、関節収束、姿勢誤差などの動的ガードで扱う。

### 用語：「崖」と「ヒューリスティック」とは

**崖（cliff）** … 機体をたたんでいくと fc は緩やかに変化するが、ある特異・不可行形状の
直前で**急に 0 近くまで落ち込む**ことがある。地形でいう「平地を歩いていて突然崖から落ちる」
イメージ。理想的には `hard_min` をこの**崖の縁の直前**に置きたい。

```text
fc
 |‾‾‾\___          ← 緩やかな変化
 |       \___
 |           \
 |            |     ← ここが「崖」（急落）
 |            └──── fc≈0（制御困難）
 +----------------- 関節角を畳む方向 →
```

**問題** … 今回は**純ホバ（外乱なし）ではどの形状でも墜落しなかった**＝崖の位置を
実測で特定できなかった。

**ヒューリスティック（heuristic）** … 「崖の縁」を測れなかったので、代わりに
**経験則（観測された fc 分布の下位5%）**で `hard_min` を仮に置いた、という意味。
物理的に「ここで制御不能」と実証した値ではなく、**統計的な当て推量**であることに注意。

→ 接触作業など**実際の外乱が大きい運用**では、その外乱条件で `disturbance_test.py` を
回して本当の崖（除去できなくなる fc）を測り、`hard_min` を測定値に置き換えるべき。
<!-- RESULTS:END -->

## 7. 再現方法

```bash
# 1. DRAGON sim を起動しホバリング
roslaunch dragon bringup.launch sim:=true headless:=true
rostopic pub -1 /dragon/teleop_command/start  std_msgs/Empty "{}"
rostopic pub -1 /dragon/teleop_command/takeoff std_msgs/Empty "{}"   # flight_state==5 まで待つ

# 2. 実験一括実行（rosbag + Part A + 解析）
rosrun dracomancer run_fc_experiment.sh ~/dracomancer_fc_experiment
#   外乱検証も行う場合:
rosrun dracomancer run_fc_experiment.sh ~/dracomancer_fc_experiment --with-disturbance

# 3. しきい値を bringup.launch / cmd トピックへ反映
```
