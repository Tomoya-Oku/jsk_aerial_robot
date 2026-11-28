# DRAGONの形状操作

形状写像の数理・制御方針は[ソフトウェア設計](software.md#全体形状写像b)、
実験条件は[評価計画](evaluation.md)、実機入力は[校正手順](hardware.md)を参照する。

## 地上デモ

Catkin workspaceでビルド・sourceしてから実行する。

```bash
roslaunch dracomancer shape_demo.launch
# CADアセンブリ表示で両腕の幾何写像を確認する。
roslaunch dracomancer shape_demo.launch assembly:=true geometry_only:=true
# 同じ肩・肘入力を繰り返す。スライダを同時操作しない。
roslaunch dracomancer shape_demo.launch motion:=true
# 新CADの左右の腕を、肩・肘・手首を連動させたランダム軌道で動かす。
roslaunch dracomancer shape_demo.launch assembly:=true geometry_only:=true motion:=true motion_pattern:=natural
# 画面なし
roslaunch dracomancer shape_demo.launch motion:=true gui:=false rviz:=false
```

Mk-II viewerとDRAGON Gazeboを起動する。離陸も偽のHOVER配信も行わない。
デモだけはHover条件を外し、既存のsimulation用preflight関節制御で地上動作させる。
通常の`bringup.launch`ではHover条件を維持する。
他のDRAGON、Gazebo、`/dragon/joints_ctrl`指令ノードを重複起動しない。

`geometry_only:=true`は地上シミュレーションで形状写像の反応を見るための設定。
このときも関節角と速度の制限を適用する。飛行可能性の評価には`geometry_only:=false`を使う。

`assembly:=true`では、左右14関節を使う両腕形状写像（`dual_shape`）が標準。
`mapping_mode:=shape`を指定すると右腕7関節を使う。
`motion_pattern:=natural`の動きは`motion_seed`（既定は27）で再現できる。
RVizには両モデル、物理端点、等弧長点、重ねた正規化中心線を表示する。水色が人体、橙色がDRAGON。
RVizでは両モデルの基準位置を2 m離して固定する。`display_gap:=2.5`のように間隔を変更できる。
DRAGONの表示にはGazeboの関節角を使う。
[両腕のランダム軌道に応じてDRAGONが形状を変える録画（斜め・真上の同時表示、MP4、36秒）](../figures/20261127_144120_shape_demo/shape-mashup.mp4)
標準設定の手先点は、左右とも同じ最新CADの各手先リンク重心を使う。
`assembly:=true`の場合は、片腕では体幹から肩への正方向を、両腕では左から右への方向を、DRAGONの尾側座標系に合わせる。

## 通常起動と設定

Mk-IIとDRAGONが稼働中なら、`shape.launch`で内部関節角の操作だけを追加できる。

```bash
roslaunch dracomancer shape.launch shape_config:=/absolute/path/to/shape.yaml
# 条件がそろってから形状操作を有効化する
rostopic pub -1 /dracomancer/enabled std_msgs/Bool 'data: true'
```

Mk-IIの起動もまとめる場合は、`bringup.launch enable_joint_angle_control:=true`に同じ設定を指定する。
設定元は`config/control/shape.yaml`。標準の`torso_point: []`では写像を有効化しない。
体幹点、人体代表点、固定軸合わせを実機に合わせて設定する。
`shape_demo.launch`と`haptics.launch simulation:=true`は、シミュレーション用に`base_link`原点の体幹代理点を明示する。
CADフレーム原点・手先重心とidentityの軸合わせも代理値で、解剖学的な実測値ではない。
最後の手首回転を反映するため、手先点を回転軸原点から離す。
URDFを変更したら人体代表点と`dragon_tail`の整合を確認する。

標準入力は`/dracomancer/measured_joint_states`、人体モデルは`/dracomancer/robot_description`。
形態指令は`control_shape.py`だけが発行する。別ノードから同じtopicへ指令しない。
対応モードは`shape`（標準）・`joint`・`dual_shape`・`dual_joint`。feasibility gateは`hold`のみ。
`shape_demo.launch mapping_mode:=dual_shape`等で選ぶ。関節角方式の既定の零点・符号はシミュレーション用。
`bringup.launch anchor_mode:=tail`は尾側点、`mid`は中央関節の位置補償を有効化する。
両腕方式の標準は`mid`。移動指令は単一の`control_position.py`が担当し、Joystick入力が零でも変形による位置差を補償する。
`enable_position_control:=true`でJoystickによる基準位置の移動も有効化する。補償はHover中に限る。
力覚には`haptics.launch`を使える。実機のARMには操作者による準備とボタン押下が必要。[修論対応](thesis.md)と[実機手順](hardware.md#実機の力覚提示)を参照する。
`enabled`の既定は`false`。`/dracomancer/enabled`に`false`を送って操作を止めると最後の指令を保持する。
現在のclutch解除は人体基準姿勢の再設定を行わない。

### `shape.launch` の引数

パッケージ名を省略したファイルパスは `dracomancer` パッケージ内のパス。トピックの既定値は `robot_ns=dracomancer`、`target_ns=dragon` の場合を示す。

| 引数 | 既定値 | 用途 |
|---|---|---|
| `robot_ns` | `dracomancer` | 操作デバイスの名前空間。計測・操作切替・診断に使用。 |
| `target_ns` | `dragon` | DRAGONの名前空間。形態指令・状態・可行性評価に使用。 |
| `shape_config` | `config/control/shape.yaml` | 関節対応、体幹点、座標合わせなどを設定するYAML。 |
| `mapping_mode` | `shape` | 形状写像の`shape`、関節角写像の`joint`、両腕版の`dual_shape`・`dual_joint`。 |
| `enabled` | `false` | `true`で起動時から形態操作を許可する。実行中は`/dracomancer/enabled`で切り替える。 |
| `enable_joint_angle_control` | `true` | 形態制御ノードを起動する。 |
| `device_joint_topic` | `/dracomancer/measured_joint_states` | 操作者側の校正済み関節計測トピック。 |
| `human_description` | `/dracomancer/robot_description` | 操作者側URDFを読み取るROSパラメータ名。 |
| `publish_joints_only_when_hovering` | `true` | DRAGONのHover中に関節指令を発行する。 |
| `max_step` | `0.015` rad | 1回に発行する関節角の変化量上限。 |
| `max_joint_velocity` | `0.1` rad/s | 関節角指令の速度上限。 |
| `enable_feasibility_gate` | `true` | 可行性評価サービスを起動し、関節指令に可行性判定を適用する。 |
| `feasibility_gate_mode` | `hold` | 判定を通過できない場合に最後の指令を保持する。実装済みの値は`hold`。 |
| `feasibility_service` | `/dragon/shape_feasibility/check_shape` | 形態制御ノードが呼ぶ可行性評価サービス名。 |
| `feasibility_rate` | `10.0` Hz | 候補形態の可行性評価頻度。発行前の再判定にも同じサービスを使う。 |
| `robot_model_plugin_name` | `dragon/full_vectoring_robot_model` | 可行性評価に用いるロボットモデルのプラグイン名。 |
| `shape_feasibility_prediction_mode` | `allocation` | 評価方法を選ぶ。選択肢は`allocation`、`controller`、`optimized_gimbal`、`model`。 |
| `force_dimensions` | `2` | 力余裕の評価次元。`2`はX・Y、`3`はX・Y・Z。 |
| `use_contact_wrench` | `false` | 可行性評価に接触力・モーメントを含める。 |
| `shape_feasibility_gimbal_feedback_topic` | `gimbals_ctrl` | ジンバル角フィードバック入力名。`target_ns`からの相対名。 |
| `shape_feasibility_baselink_rpy_topic` | `final_target_baselink_rpy` | 基部の目標RPY入力名。`target_ns`からの相対名。 |
| `shape_feasibility_baselink_odom_topic` | `uav/baselink/odom` | 基部オドメトリ入力名。`target_ns`からの相対名。 |
| `shape_feasibility_use_gimbal_feedback` | `true` | ジンバル角フィードバックを評価に使用する。 |
| `shape_feasibility_use_baselink_rpy_feedback` | `true` | 基部の目標RPYを評価に使用する。 |
| `shape_feasibility_use_baselink_odom_feedback` | `true` | 基部オドメトリを評価に使用する。 |
| `shape_feasibility_feedback_timeout` | `0.25` 秒 | 評価に使うフィードバックの鮮度上限。 |
| `shape_feasibility_allocation_refine_max_iteration` | `5` | `allocation`で推力・ジンバル角を求める反復回数上限。 |
| `shape_feasibility_allocation_refine_threshold` | `0.0001` | `allocation`の反復で収束を判定する差分閾値。 |
| `force_radius_threshold` | `0.108990` N | 形態指令の保持判定に使う力余裕の閾値。 |
| `torque_radius_threshold` | `0.015400` N·m | 形態指令の保持判定に使うトルク余裕の閾値。 |
| `force_radius_recover_threshold` | `0.249220` N | 保持状態から復帰するための力余裕の閾値。 |
| `torque_radius_recover_threshold` | `0.278159` N·m | 保持状態から復帰するためのトルク余裕の閾値。 |
| `enable_volume_radius_monitor` | `true` | 飛行中の力・トルク余裕を購読し、監視値を配信するノードを起動する。 |
| `enable_shape_safety` | `true` | 監視ノードが余裕に応じた安全度を計算する。 |
| `missing_inradius_scale` | `1.0` | 余裕の入力が欠落または期限切れの場合の安全度。 |
| `min_safety_scale` | `0.0` | 安全度の下限。いずれかの余裕が低い側の閾値以下ならこの値を使う。 |
| `force_inradius_min` | `0.249220` N | 監視時に安全度が1に達する側の力余裕閾値。 |
| `torque_inradius_min` | `0.278159` N·m | 監視時に安全度が1に達する側のトルク余裕閾値。 |
| `force_inradius_hard_min` | `0.108990` N | 監視時に安全度を下限にする力余裕閾値。 |
| `torque_inradius_hard_min` | `0.015400` N·m | 監視時に安全度を下限にするトルク余裕閾値。 |
| `inradius_timeout` | `0.5` 秒 | 監視に使う力・トルク余裕の鮮度上限。 |
| `safety_log_period` | `1.0` 秒 | 余裕低下ログの出力間隔。 |
| `enable_fc_min_lowpass` | `true` | 力・トルク余裕に一次ローパスをかけるノードを起動する。 |
| `fc_min_lowpass_cutoff_hz` | `2.0` Hz | 一次ローパスの遮断周波数。 |
| `monitor_force_inradius_topic` | `/dragon/debug/fc_f_min_filtered` | 監視用の力余裕入力。ローパス無効時の既定は`/dragon/debug/fc_f_min`。 |
| `monitor_torque_inradius_topic` | `/dragon/debug/fc_t_min_filtered` | 監視用のトルク余裕入力。ローパス無効時の既定は`/dragon/debug/fc_t_min`。 |

## 実装の入口

| ファイル | 役割 |
|---|---|
| `scripts/control/shape.py` | URDF FK、中心線・関節角写像、最適化、速度制限 |
| `src/kinematics/shape_kernel.cpp` | 最適化で繰り返すDRAGONの順運動学と中心線サンプリングを一括で計算 |
| `scripts/teleop/control_shape.py` | 入力検証、可行性ゲート、診断・Marker、単一の形態指令発行 |
| `scripts/simulation/backend.py motion` | デモの再現用入力。DRAGONへは指令しない |
| `config/control/shape.yaml` | 代表点・軸合わせ・最適化設定 |

初期化は新鮮な機体実測姿勢・入力・設定・可行性を確認するまで待機する。
最適化失敗、入力欠損・古い時刻・未来時刻、不正値、時間超過、サービス障害では指令を進めない。
候補を検査し、速度制限後の発行形態も再検査する。検査済み候補の再利用は同一候補に限定する。

## 診断とログ

| Topic | 内容 |
|---|---|
| `/dracomancer/candidate/joint_target` | ゲート前の候補`q_ref` |
| `/dragon/joints_ctrl` | 発行指令`q_command` |
| `/dragon/joint_states` | 機体の実関節フィードバック |
| `/dracomancer/shape/rmse` | 人体対発行指令の正規化・重み付き位置RMSE |
| `/dracomancer/shape/diagnostics` | JSON：候補・指令、誤差、処理時間、反復、ok・status・gate・updated |
| `/dracomancer/shape/markers` | 物理端点と正規化中心線のMarkerArray |
| `/dracomancer/shape_control_error` | 関節候補−発行指令の6値。形状RMSEとは別指標 |

`seconds`は人体FKを含む直前の最適化更新時間。`updated=true`を対象に集計する。
`control_seconds`は写像・ゲート・最終検査までで、診断・Marker配信とROS sleepを含まない。
入力異常後のRMSE・Markerが最後の有効入力を参照する場合があるため、`status`・`ok`と併せて読む。

```bash
rosbag record -O shape.bag /dracomancer/measured_joint_states /dragon/joint_states \
  /dragon/joints_ctrl /dracomancer/candidate/joint_target \
  /dracomancer/shape/diagnostics /dracomancer/shape/rmse \
  /dracomancer/shape_control_error /dragon/flight_state
```

## 評価時の注意

`figures/scripts/dragon_fk.py`は過去の形態目標到達タスクの`E_s`に対応する作図用FK。
既存CSVの意味を変えず、Human–Robot形状RMSEとは区別する。
中心線に現れない軸回旋・手先姿勢は復元できず、CAD関節と人体関節が同じ座標であるとも扱わない。
速度制限は発行目標の制限で、実関節速度・経路中間点・動的安定性・衝突回避の保証ではない。
同期feasibility RPCと描画・他プロセスの負荷を含む周期時間は実環境で測定する。
再検証・実機校正・力覚接続の残作業は[roadmap.md](roadmap.md)を参照する。
