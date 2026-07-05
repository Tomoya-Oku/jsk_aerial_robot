# 形態目標到達タスク (shape_task)

Dracomancer（上肢外骨格デバイス）で DRAGON の形態を操作し、シミュレータ上に表示された
目標形態シャドウへ現在形態を一致させる実験のためのノード群・解析スクリプト。

既存の制御ノード（`control_joint_angle.py` など）・launch・メッセージ定義は一切変更していない。
実験ノードは既存トピックを購読・イベント配信するだけの追加構成である。

## ファイル構成

| ファイル | 役割 |
| --- | --- |
| `target_manager.py` | 試行管理（目標選択・成功判定・タイムアウト・start/stop/reset/next サービス） |
| `shadow_visualizer.py` | 目標形態を半透明シャドウ（RViz MarkerArray）として表示 |
| `task_recorder.py` | 試行中の全データを raw CSV に記録し、試行終了時に指標を自動計算 |
| `compute_metrics.py` | 評価指標の計算（オフライン CLI としても使用可） |
| `keyboard_baseline.py` | キーボード比較条件（同一ゲート・同一成功条件・同一ログ） |
| `dummy_robot.py` | dry-run 用の擬似 DRAGON（Gazebo/実機不要） |
| `dragon_fk.py` | E_s・シャドウ用の簡易 DRAGON FK（共有モジュール） |
| `../../config/shape_task/targets.yaml` | 目標形態リスト（L-like / Zigzag-like / Spiral-like） |
| `../../config/shape_task/experiment.yaml` | しきい値・トピック名・試行管理などの全設定 |
| `../../launch/shape_task.launch` | 実験ノード群の起動 |
| `../../figures/plot_task.py` | 結果ディレクトリからの図生成 |

## 実行手順

### A. Dracomancer 条件（Gazebo シミュレーション）

```bash
# 端末1: DRAGON シミュレータ
roslaunch dragon bringup.launch real_machine:=false simulation:=true headless:=false

# 端末2: Dracomancer デバイス（実機なしなら rm:=false でスライダ入力）
roslaunch dracomancer bringup.launch rm:=false sim:=true
roslaunch dracomancer teleoperation.launch

# 端末3: 形態目標到達タスク
roslaunch dracomancer shape_task.launch

# 離陸（rostopic pub /dragon/teleop_command/takeoff など既存手順）後、
# teleoperation モードへ切替
rostopic pub -1 /dracomancer/teleop_mode std_msgs/String "data: 'teleoperation'"

# 試行開始
rosservice call /shape_task_manager/start
```

RViz では `MarkerArray` ディスプレイを追加し、トピック
`/dracomancer/shape_task/shadow` を指定する（Fixed Frame は `world` のままでよい。
シャドウは既定で `dragon/link1` フレームに貼られ、E_s の定義と同じ座標系で重なる）。
表示先は DRAGON 側 RViz（`dragon/config/rviz_config`）を想定する。Dracomancer 側 RViz は
Fixed Frame が `dracomancer/base_link` のため、`dragon/link1` と TF がつながらず表示できない。
`shape_task.launch` 起動直後は最初の目標を灰色のプレビューとして表示し、試行開始後に状態色へ切り替わる。
TF 切り分けだけしたい場合は `roslaunch dracomancer shape_task.launch shadow_frame:=world`
で world 固定表示にできる。

### B. keyboard 比較条件

Dracomancer 入力の代わりにキーボードで 6 関節を操作する。
`teleoperation.launch` は起動しない（`joints_ctrl` の競合を避ける）。

```bash
# 端末1: DRAGON シミュレータ（A と同じ）
# 端末2: 安全ゲートのサーバ + タスクノード
roslaunch dracomancer shape_task.launch launch_feasibility:=true
# しきい値配信が必要なら（bringup.launch を使わない場合）:
rosrun dracomancer volume_radius_monitor.py

# 端末3: キーボード入力（stdin が必要なので必ず rosrun で起動する）
rosrun dracomancer keyboard_baseline.py
```

キー操作: `1`-`6` 関節選択 / `+` `-` 増減 / `[` `]` ステップ半減・倍増 / `r` 現在形態にリセット / `h` ヘルプ。

keyboard 条件は Dracomancer 条件と同一の hold 型フィージビリティゲート
（`/dragon/shape_feasibility/check_shape`、[hard_min, min] しきい値トピック、ヒステリシス）、
同一の `max_step` レート制限、同一のホバリングゲートを `keyboard_baseline.py` 内に再実装しており、
出力トピック（`/dragon/joints_ctrl`、`/dracomancer/shape_control_error`、
`/dracomancer/candidate/fc_*`）も同じため、記録・評価・可視化は完全に共通である。
差は入力デバイスのみ。

### C. dry-run（Gazebo・実機・Dracomancer なし）

```bash
# 端末1: 擬似 DRAGON 込みでタスクを起動（fc は安全な定数を配信）
roslaunch dracomancer shape_task.launch dryrun:=true auto_start:=true

# 端末2: キーボードで形態を操作（ゲートなし）
rosrun dracomancer keyboard_baseline.py _enable_feasibility_gate:=false
```

Dracomancer 側のダミー入力で試す場合は `dracomancer bringup.launch rm:=false sim:=true`
（GUI スライダ → `/dracomancer/joint_states`）+ `teleoperation.launch
enable_feasibility_gate:=false enable_link4_anchor:=false` を併用する。
`teleoperation.launch` 単体で servo から `/dracomancer/joint_states` を作る場合だけ
`enable_servo_to_joint_states:=true` を付ける。

### 試行管理サービス

```bash
rosservice call /shape_task_manager/start   # 試行開始（以後は自動で次の試行へ進む）
rosservice call /shape_task_manager/stop    # 実行中の試行を中断
rosservice call /shape_task_manager/next    # 現在の試行をスキップして次の目標へ
rosservice call /shape_task_manager/reset   # スケジュールを最初の目標に戻す
```

## 成功条件と時間の扱い

以下すべてを `t_hold` 秒維持で成功。`timeout` 秒で失敗（experiment.yaml で変更可能）。

- `E_q < E_q_threshold` : RMS(q_current − q_star)（rad、既定 0.15）
- `E_s < E_s_threshold` : リンク端点（joint1/2/3・link4先端）位置誤差の RMS（m、既定 0.05。`E_s_normalize: true` で全長正規化）
- `mu >= 0` : `mu = min((r_f − r_f_hard)/(r_f_safe − r_f_hard), (r_τ − r_τ_hard)/(r_τ_safe − r_τ_hard))`

経過時間・保持時間・タイムアウトはすべて **ROS time**（Gazebo では sim time）。
結果ディレクトリ名と `wall_time` 列のみ wall time。

## 出力ログ

既定の保存先は `~/dracomancer_shape_task/YYYYMMDD_HHMMSS/`（`results_root` / `results_dir` で変更可能）。

```
results/YYYYMMDD_HHMMSS/
  session.yaml            # 有効設定・目標リストのスナップショット
  trial_001_raw.csv       # 時系列 raw ログ（record_rate = 40 Hz）
  trial_001_metrics.csv   # 試行の評価指標
  summary_metrics.csv     # 全試行の指標一覧（1試行1行）
  plots/                  # figures/plot_task.py の出力（session_end 時に自動生成）
```

raw CSV 列: `time, target_name, q_star_0..5, q_current_0..5, q_target_0..5,
q_candidate_0..5, q_exo_0..6, r_f, r_tau, mu, gate_rejected, E_q, E_s` +
診断用の `gate_state, cand_r_f, cand_r_tau, safety_scale, flight_state, wall_time`。
未受信（`stale_timeout`=1 s 以上更新なし）の値は NaN、`gate_rejected` 不明時は空欄。

## 評価指標（compute_metrics.py）

| 指標 | 計算方法 |
| --- | --- |
| `success` / `completion_time` | target_manager の判定（開始→保持完了までの時間）。イベント欠損時は raw から再計算 |
| `final_E_q` / `final_E_s` | 成功時は達成時点、失敗時は最終サンプルの値 |
| `total_input_I` | Σ‖q_exo[k] − q_exo[k−1]‖（rad） |
| `response_delay_T_d` | \|dq_exo/dt\| と \|dq_target/dt\| の相互相関が最大となるラグ（0〜2 s） |
| `mu_min` | 試行中の mu の最小値 |
| `gate_count_N_s` | gate_rejected の False→True 遷移回数 |
| `gate_time` | gate_rejected=True の累積時間（s） |
| `path_efficiency` | ‖q_star − q_current[0]‖ / q_current の関節空間軌跡長 |

オフライン再計算: `./compute_metrics.py trial_001_raw.csv`、
図の再生成: `rosrun dracomancer plot_task.py <results_dir>`（`figures/plot_task.py`）。

## 未接続・未確認項目

- `q_candidate` はトピックとして存在しないため `q_target + shape_control_error` から再構成している
  （`control_joint_angle.py` は同一周期で両者を publish するため近似は正確）。
- `gate_rejected` は候補予測 fc (`/dracomancer/candidate/fc_*`) と hard しきい値から導出しており、
  `control_joint_angle.py` 内部のヒステリシス保持状態そのものではない（保持解除は recover しきい値なので、
  境界付近では内部状態と数サンプルずれうる）。
- 実測 `r_f`/`r_tau` は `/dragon/debug/fc_f_min_filtered` / `fc_t_min_filtered`
  （`fc_min_lowpass.py` 有効時）。低域通過を使わない構成では experiment.yaml で `_filtered` なしに変更する。
- dry-run では `q_exo`・fc 系トピックが存在せず NaN で記録される
  （`require_safety_margin: false` にしないと成功判定が通らない）。
- 実機 DRAGON・実機 Dracomancer での動作は未検証（シミュレーション前提で作成）。

## keyboard 比較を拡張する場合

- 入力デバイス条件を増やすには「`/dragon/joints_ctrl`・`/dracomancer/shape_control_error`・
  `/dracomancer/candidate/fc_*` を同じ意味で publish するノード」を追加するだけでよい。
  タスク管理・記録・評価・可視化は入力に依存しない。
- 条件ラベルを結果に残すには試行ごとに `results_dir` を分ける
  （例: `roslaunch dracomancer shape_task.launch results_dir:=~/exp/keyboard_01`）。
- ゲート仕様を変える場合は Dracomancer 条件 (`teleoperation.launch` の引数) と
  `keyboard_baseline.py` のパラメータを必ず同時に揃えること。
