# fc 安全しきい値の較正実験

DRAGON の **可制御レンチ内接半径** `fc_f_min` / `fc_t_min` に対する
Dracomancer 安全ゲートのしきい値 `[hard_min, min]`（[volume_radius_monitor.py](../scripts/volume_radius_monitor.py)）を、
**データに基づいて**決めるための実験手順と結果を記録する。

関連: [dracomancer_system.md](dracomancer_system.md)（安全スケールの定義）。

## 1. 背景：fc とは何か、外乱はどう関わるか

`fc_f_min` / `fc_t_min` は [robot_model.cpp](../../../aerial_robot_model/src/model/base_model/robot_model.cpp#L495-L603) で計算される、
**可制御力／トルク凸包の内接半径**である。ロータ配置とスラスト制約のみで決まり、
**機体形状（内部関節角）の関数**である。

```
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

```
hard_min = max(0, percentile(fc, hard_pct) × safety)   # 危険下限＝崖の手前
min      = median(fc) × min_frac                        # 完全安全点＝典型値の一定割合
```

- `hard_min`：到達しうる形状での fc 下側分位（既定 p5、崖の直上）。Part B の安全係数を掛ける。
- `min`：典型（中央値）fc の `min_frac`（既定 0.8）。これ以上で安全スケール=1.0。
- **力・トルクは桁が違うため独立に決める**。

## 6. 結果（このリポジトリでの実測）

> 取得日: 2026-06-21 / DRAGON quad sim / ホバ高度 1.0 m
> 生成物: `~/dracomancer_fc_experiment/`（`fc_sweep.csv`, `fc_sweep.bag`, `proposed_thresholds.txt`）

<!-- RESULTS:BEGIN -->
### Part A：fc 分布（83 形状サンプル）

| 量 | min | p5 | p25 | median | p75 | max |
| --- | --- | --- | --- | --- | --- | --- |
| `fc_f_min` | 0.037 | 0.061 | 0.103 | **0.185** | 0.268 | 1.418 |
| `fc_t_min` | 0.0007 | 0.0016 | 0.012 | **0.048** | 0.279 | 1.921 |

- ノミナル円形姿勢（離陸形状）は `fc_f≈0.09 / fc_t≈0.005` と**分布の下端**。特にトルクはほぼ 0 で、円形姿勢はトルク可制御性が乏しい。
- 全サンプルで**ホバリングは破綻せず**（高度1.0m維持）。純ホバではどの形状も保持可能。

### Part B：外乱除去（ノミナル形状・lateral force）

| 印加力[N] | 2 | 4 | 6 | 9 | 12 |
| --- | --- | --- | --- | --- | --- |
| 最大COG誤差[m] | 0.07 | 0.13 | 0.20 | 0.31 | 0.40 |
| 制御保持 | ✔ | ✔ | ✔ | ✔ | ✔ |

- 誤差は印加力にほぼ**線形**（≈0.033 m/N の定常偏差）、12 N でも喪失せず。
- **重要**：`fc_f_min≈0.09` に対し 12 N を余裕で除去 → **fc は絶対 N ではなく正規化された相対量**。
  したがって `hard_min` を「絶対外乱[N]」で決めるのは不適切で、**fc 分布の相対位置**で決めるのが妥当。

### 決定したしきい値（データ駆動）

`analyze_fc_data.py --hard-pct 5 --min-frac 0.8` の提案を丸めて採用:

| 軸 | hard_min | min | 根拠 |
| --- | --- | --- | --- |
| force | **0.06** | **0.15** | hard=p5、min=median×0.8 |
| torque | **0.002** | **0.04** | hard=p5、min≈median×0.8 |

- これにより典型形状で安全スケール≒1、下位 ~25% の低 fc 形状で連続的に抑制がかかる。
- 旧既定 `force[0.0, 0.6] / torque[0.0, 0.1]` は min が分布の最大付近で**過剰に保守的**（常時 scale<1）だった。
- 純ホバでは崖が現れないため `hard_min` は分布下側分位に基づく**ヒューリスティック**。接触作業など想定外乱が大きい運用では Part B を該当外乱条件で再実施し `hard_min` を引き上げること。
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
