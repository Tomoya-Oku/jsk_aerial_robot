# Dracomancerパッケージ

このファイルは `robots/dracomancer/` を編集する際に Codex/ClaudeなどのAIエージェント が参照する前提知識です。
ユーザーのグローバル指示（日本語での結論先出し・小さな変更・確認の徹底など）に従うこと。

## このパッケージは何か

Dracomancer は **DRAGON を遠隔操作する上肢外骨格（エクソスケルトン）デバイス**であり，操縦者の腕の関節角とジョイスティック（未実装）、背中の IMU を読み取り、DRAGON の**移動指令（FlightNav）** と **形状指令（joints_ctrl）** に変換する。

- 実体は主に `scripts/` 配下の **Python ノード**。
- 例外として `src/shape_feasibility_node.cpp` は実装済みの C++ サービスノード（pluginlib で DRAGON モデルを読み込み fc 半径を予測）。
- `src/` の `dracomancer.cpp`, `model/`, `control/` と `include/` は**空のプレースホルダ**。
  C++ プラグイン（`plugins/*.xml`）は宣言のみで中身がない。安易に「実装済み」と扱わないこと。

## ファイル構成と役割

| パス | 役割 |
| --- | --- |
| `scripts/device_io/calibrate_joystick.py` | 操縦桿の生値較正 → `/dracomancer/joystick/calibrated` |
| `scripts/device_io/convert_servo_to_joint_states.py` | サーボ tick → rad の関節状態。ID0〜6 を固定マッピング |
| `scripts/device_io/publish_fake_joint_states.py` | rm:=false 用のダミー関節状態 publisher |
| `scripts/control/control_position.py` | 操縦桿(+IMU) → DRAGON 移動指令 `/<robot>/uav/nav` |
| `scripts/control/control_orientation.py` | 操縦桿 → 姿勢指令 `/<robot>/final_target_baselink_rpy` |
| `scripts/control/control_joint_angle.py` | 腕関節 → DRAGON 形状指令 `/<robot>/joints_ctrl`（候補姿勢のフィージビリティで変形可否を判定） |
| `scripts/control/control_haptic_feedback.py` | 形状抑制量 → Dracomancer 力覚提示 |
| `src/shape_feasibility_node.cpp` | 候補リンク角の force/torque volume 半径を DRAGON モデルで予測するサービス（C++） |
| `scripts/safety/volume_radius_monitor.py` | しきい値 pub/sub・ライブ安全スケール算出（bringup で常時起動） |
| `scripts/safety/fc_min_lowpass.py` | 実測 fc_f_min/fc_t_min のローパス（ns=dragon、危険判定の平滑化） |
| `scripts/experiments/` | fc しきい値較正の実験ツール（収集/外乱/解析、[docs/fc_threshold_calibration.md](docs/fc_threshold_calibration.md)） |
| `scripts/shape_task/` | 形態目標到達タスク実験（試行管理/シャドウ表示/記録/評価/キーボード比較、[scripts/shape_task/README.md](scripts/shape_task/README.md)。設定は `config/shape_task/`、起動は `launch/shape_task.launch`、図生成は `figures/plot_task.py`） |
| `launch/bringup.launch` | デバイス本体（URDF/TF/サーボ橋渡し/FC/安全半径モニタ） |
| `launch/teleoperation.launch` | 位置・姿勢・関節角制御ノード群 |
| `launch/haptics.launch` | 力覚提示ノード |
| `launch/rviz.launch` | GUI PCでRVizだけを起動 |
| `launch/include/sensors.launch.xml` | spinal bridge（FC/IMU）と imu 設定読込 |
| `config/` | Servo/joystick_calibration/navigation 等の設定 |
| `docs/` | 仕様ドキュメント（中粒度・Mermaid 図中心） |

## 重要な前提・規約

- **編集はできるだけ `dracomancer/` 内にとどめる**（ユーザー指示）。
  共通機能（`aerial_robot_model` 等）の変更が必要なときは事前に相談する。
- **トピック名**
  - 操縦桿: `/joystick/raw`(Int16MultiArray) → `/dracomancer/joystick/calibrated`(Float32MultiArray)
  - サーボ: `/dracomancer/servo/states`(spinal/ServoStates) → `/dracomancer/joint_states`(JointState)
  - IMU: `/dracomancer/imu`（spinal/Imu, `quaternion[x,y,z,w]`）。namespace `dracomancer` 下の spinal bridge が publish。
  - DRAGON 出力: `/dragon/uav/nav`(FlightNav), `/dragon/joints_ctrl`(JointState)
- **spinal.msg** は `convert_servo_to_joint_states.py` / `control_position.py` / `control_haptic_feedback.py` で使用。`package.xml` に `spinal` run_depend を宣言済み。
- **rm / sim** が通常の起動引数。`real_machine` / `simulation` は旧互換エイリアス。
- Khadas など表示なし環境を想定し、RViz は既定で起動しない（`headless` 既定 True）。
- `teleop_mode` は `startup` / `teleoperation`（`teleop` は別名、launch 既定は `startup`）。`teleoperation` では形状（腕写像）が有効。移動（操縦桿）は既定 OFF（`enable_position_control:=false`。`control_position.py` の FlightNav が `POS_VEL_MODE`＋`target_pos` 未設定で落下するため。README「既知の課題」参照）。実行中は `/dracomancer/teleop_mode` (`std_msgs/String`) で切り替える。

## FlightNav（aerial_robot_msgs）の要点

移動指令で使う主フィールド:
- `control_frame`: `WORLD_FRAME(0)` / `LOCAL_FRAME(1)`
- `target`: `BASELINK(0)` / `COG(1)`
- `*_nav_mode`: `POS_VEL_MODE(4)` などを使用。control_position は速度指令（`target_vel_*`）を送る。

## control_position.py（移動制御）の設計

`teleoperation` モードでのみ `/dragon/uav/nav` を送る。IMU（spinal 由来。操作者の背中に **90度傾けて** 搭載）の姿勢で、操縦桿の移動方向を
**操作者の向きに対する相対移動**に変換する。重心/ベースリンク切替も持つ。

主なパラメータ:
- `nav_target`: `cog`(既定) / `baselink` → FlightNav.target。
- `direction_mode`: `none` / `yaw`(既定) / `yaw_pitch` / `full`。
- `imu_topic`: 既定 `/dracomancer/imu`。
- `imu_mount_rpy`: IMU 取付け補正 [roll,pitch,yaw] rad（既定 `[0, -π/2, 0]`、実機で要調整）。
- `recapture_on_hover`: ホバ開始時に中立姿勢を再キャプチャ（既定 true）。
- `~recapture_neutral`(std_msgs/Empty): 中立再キャプチャのトリガ。
- `axis_x/y/z`: 既定 `0/1/2`。既定較正は2軸なので、3軸目がなければZ速度は0。

## control_joint_angle.py（形状制御）の設計

- `startup`: `startup_pose`（既定 `[0, pi/2, 0, pi/2, 0, pi/2]`）を保持。
- `teleoperation`: Dracomancer 腕関節を DRAGON 関節へマッピングし、**候補姿勢のフィージビリティで変形可否を判定**する（移動は `control_position.py` が同時に担当）。マッピングは `~mapping_mode` で切替（`distal`=既定: 腕関節を DRAGON 関節へ**絶対角で一致**（中立記録なし。既定対応: 手首屈曲→上腕ロール+前腕ロール角に応じてjoint1_pitch/joint1_yawへ配分、肘屈曲→上腕ロール角に応じてjoint2_pitch/joint2_yawへ配分、肩屈曲→joint3_pitch、肩内外転→joint3_yaw。`clamp(sign*scale*source + offset)`、`distal_signs` 既定 `[1,-1,1,1,-1]`・`distal_offsets` 既定 `[0,0,0,π/2,0]`（device は肩屈曲を負で測るため joint3_pitch は sign=+1+offset=π/2 で 90°→0°。rosbag 解析で確定）、平行リスト `distal_source_joints`/`distal_target_joints`/`distal_signs`/`distal_scales`/`distal_offsets` で設定。`enable_wrist_roll_switching` 既定ONでは上腕+前腕ロール総和から `wrist_roll_parallel_offset` を引いた絶対値45°以下で手のひら水平に近い側として手首屈曲をjoint1_pitch、それ以外で手のひら垂直に近い側としてjoint1_yawへ入れる。`enable_elbow_roll_switching` 既定ONでは上腕ロール絶対値15°以下で肘屈曲をjoint2_pitch、それ以外でjoint2_yawへ入れる。旧名 `elbow_only` は後方互換エイリアス） / `joint_pairing`: 3屈曲関節→3 yaw・pitch=0で平面保持 / `geometric`: 腕FK→面内yaw・面外pitch分解）。詳細は [README.md](README.md) の関節マッピング節。
- **予測フィージビリティ・ゲート**: 候補 DRAGON 形状を `shape_feasibility_node`（C++、`dragon/full_vectoring_robot_model` を pluginlib で読み込み、`ns=dragon` で起動）の `check_shape` サービスに渡し `fc_f_min`/`fc_t_min` を予測。force・torque 両方が下限しきい値以上なら変形（採用・記憶）、未満／サービス失敗なら直前可行姿勢で停止。**既定 ON（`enable_feasibility_gate:=true`）**。予測fcは既定 `shape_feasibility_prediction_mode=optimized_gimbal` で、DRAGONモデルのジンバル処理済み状態から `calcFeasibleControlFxyDists()` / `calcFeasibleControlTDists()` を再評価する。将来controller割当器を共有する場合は `controller` モードに差し替える。モデル/設定によって予測 fc が過小になり全変形を却下する場合は、検証用に `enable_feasibility_gate:=false` で一時無効化できる。下限しきい値は `*_volume_radius_threshold`（`[hard_min, min]` の `hard_min`）を購読、未受信時は `force_radius_threshold`/`torque_radius_threshold` パラメータ。
- **ライブ監視（情報提供）**: `volume_radius_monitor.py`（bringup.launch）が実機現在状態の fc からライブスケールを `/dracomancer/dragon_shape_safety_scale` に publish（web UI 用、ゲートとは独立）。しきい値の所有・pub/sub もここ。
- **link4 アンカー**（既定 ON `enable_link4_anchor`）: 関節操作中も DRAGON の link4 位置をワールドにおおよそ固定するため、ホバー開始時に link4 TF をキャプチャし、毎周期 `joints_ctrl` と同じ目標関節角から `fc→link4(q)` を最小FKで計算して `T_anchor·T_cog→link4(q)⁻¹` を逆算する。`link4_anchor_mode=position_only` が既定で、COG位置だけを `uav/nav` POS_MODE へ送る。`link4_anchor_mode=full` は baselink姿勢も補償するが、姿勢failsafeに近づきやすいため明示指定時のみ使う。移動制御（`enable_position_control`）とは `uav/nav` が競合するため併用不可。詳細は [docs/link4_anchor.md](docs/link4_anchor.md)。
- **link4 body step scaling / safety**（既定 ON）: `max_step` 後の候補関節角から必要なCOG位置変化量を予測し、`link4_anchor_max_body_pos_rate` を超える場合は関節ステップ全体を縮小する。`full` モードでは baselink姿勢変化量も `link4_anchor_max_body_rpy_rate` で抑える。安全判定は position_only では COG高度 + ホバー開始時COGからの水平距離、full ではさらに baselink roll/pitch を検査する。TFが取得できずbody補償を計算できない周期は直前姿勢を保持する。
- **baselink roll マッピング**（distal 標準・既定 ON `enable_baselink_roll_mapping`）: `link4_anchor_mode=full` 時のみ、`upper_arm_external_internal_rotation_joint` と `wrist_supination_joint` の中立値からの差分和を link4 アンカーで算出した baselink roll に加算する。中立値は最初の受信、ホバー開始、link4アンカー再取得でキャプチャする。
- 位置・姿勢・関節角操作はいずれもホバリング（`flight_state == hover_flight_state`、既定5）時のみ出力する（`publish_joints_only_when_hovering` 既定 true）。
- DRAGON の nav は `HOVER_STATE == 5` でのみ受理される。link4アンカー補償も `hover_flight_state` 既定 5 のときだけ出すこと。`flight_state>=4` のような判定は `LAND_STATE`/`PRE_LAND_STATE` を誤って含む。
- `shape_feasibility_node` は DRAGON のモデル/パラメータを使うため **DRAGON 起動が前提**。`dragon` を run_depend に追加済み。

### 数学的に重要な事実（変更時の注意）

- 操作者の向きは `q_wo = q_spinal ⊗ q_mount`（操作者ボディ→ワールド）で表す。
- 起動時に中立 `q_wo0` をキャプチャし、ワールド相対回転
  `q_rel = q_wo ⊗ q_wo0⁻¹` を用いる。
- **`yaw` モードでは取付け補正 `q_mount` がキャンセルされる**
  （`q_rel = q_spinal ⊗ q_spinal0⁻¹` となるため）。ヨー操舵は取付け角に依存しない。
  → `imu_mount_rpy` が効くのは `yaw_pitch` / `full`（操作者の前方軸=ボディx が必要）のみ。
- spinal は磁気を使わないと **yaw がドリフト**する。だから絶対値ではなく
  中立キャプチャ（相対）方式を採用している。`recapture_on_hover` で離陸直前に取り直す。
- 四元数は `[x,y,z,w]` 規約。ヘルパー（quat_mult 等）は numpy 自前実装で外部依存なし。

## ビルド / 動作確認

```bash
catkin build aerial_robot_model dracomancer
source devel/setup.bash
python3 -m py_compile robots/dracomancer/scripts/*.py   # 構文チェック
```

実機がなくても Python の構文チェックと、回転ロジックの数値検証は可能。
ROS ノードの起動には spinal/aerial_robot 一式が必要。

## ドキュメント

実装を変えたら関連ドキュメントを更新すること（中粒度・Mermaid 図・日本語）。

- `README.md`: システム仕様、モード、トピックフロー、関節マッピング、安全機構、起動引数（旧 `docs/dracomancer_system.md` を統合）
- `docs/`: 個別トピックの補足（例: `fc_threshold_calibration.md`、`link4_anchor.md`(distal の link4 アンカー)）

### 使用機材

- 親機PC: Ubuntu22.04・ROS1 (noetic)
- 子機PC (Dracomancer背中部に搭載): Khadas VIM4
- マイコン: Spinal
- 3DCAD: SolidWorks2024
- Dracomancer
  - サーボモータ: DYNAMIXEL XL430-W250-T * 7
  - リンク: PLA (Bambu Lab)

## マッピング方針

- Wrist flexion/extension: Upper-arm + forearm roll angle switches Joint1-Pitch / Joint1-Yaw (default switch boundary: 45 deg from the palm-horizontal reference)
- Wrist radial/ulnar deviation: Joint1-Yaw (rotated with wrist flexion when wrist roll switching is enabled)
- Forearm pronation/supination + Shoulder internal/external rotation: Baselink-Roll delta in `link4_anchor_mode=full`
- Elbow flexion/extension: Upper-arm roll angle switches Joint2-Pitch / Joint2-Yaw (default switch boundary: 15 deg)
- Shoulder flexion/extension: Joint3-Pitch
- Shoulder abduction/adduction: Joint3-Yaw
- Joint2-Pitch is used by elbow roll switching; do not treat it as unused redundancy in the current mapping.
