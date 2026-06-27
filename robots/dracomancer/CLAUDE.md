# CLAUDE.md — dracomancer パッケージ

このファイルは `robots/dracomancer/` を編集する際に Claude が参照する前提知識です。
ユーザーのグローバル指示（日本語での結論先出し・小さな変更・確認の徹底など）に従うこと。

## このパッケージは何か

Dracomancer は **DRAGON を遠隔操作する上肢外骨格（エクソスケルトン）デバイス**。
オペレータの腕の関節角と操縦桿、背中の IMU を読み取り、DRAGON の
**移動指令（FlightNav）** と **形状指令（joints_ctrl）** に変換する。

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
- `teleop_mode` は `startup` / `precision` / `wide`。実行中は `/dracomancer/teleop_mode` (`std_msgs/String`) で切り替える。

## FlightNav（aerial_robot_msgs）の要点

移動指令で使う主フィールド:
- `control_frame`: `WORLD_FRAME(0)` / `LOCAL_FRAME(1)`
- `target`: `BASELINK(0)` / `COG(1)`
- `*_nav_mode`: `POS_VEL_MODE(4)` などを使用。control_position は速度指令（`target_vel_*`）を送る。

## control_position.py（移動制御）の設計

`wide` モードでのみ `/dragon/uav/nav` を送る。IMU（spinal 由来。操作者の背中に **90度傾けて** 搭載）の姿勢で、操縦桿の移動方向を
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
- `precision`: Dracomancer 腕関節を DRAGON 関節へマッピングし、**候補姿勢のフィージビリティで変形可否を判定**する。マッピングは `~mapping_mode` で切替（`joint_pairing`=既定: 3屈曲関節→3 yaw・pitch=0で平面保持 / `geometric`: 腕FK→面内yaw・面外pitch分解）。詳細は [docs/dracomancer_system.md](docs/dracomancer_system.md) の関節マッピング節。
- `wide`: `wide_hold_pose`（既定 `startup_pose`）を保持し、移動は `control_position.py` に任せる。
- **予測フィージビリティ・ゲート（主機構）**: 候補 DRAGON 形状を `shape_feasibility_node`（C++、`dragon/full_vectoring_robot_model` を pluginlib で読み込み、`ns=dragon` で起動）の `check_shape` サービスに渡し `fc_f_min`/`fc_t_min` を予測。force・torque 両方が下限しきい値以上なら変形（採用・記憶）、未満／サービス失敗なら直前可行姿勢で停止。下限しきい値は `*_volume_radius_threshold`（`[hard_min, min]` の `hard_min`）を購読、未受信時は `force_radius_threshold`/`torque_radius_threshold` パラメータ。
- **ライブ監視（情報提供）**: `volume_radius_monitor.py`（bringup.launch）が実機現在状態の fc からライブスケールを `/dracomancer/dragon_shape_safety_scale` に publish（web UI 用、ゲートとは独立）。しきい値の所有・pub/sub もここ。
- 位置・姿勢・関節角操作はいずれもホバリング（`flight_state>=4`）時のみ出力する（`publish_joints_only_when_hovering` 既定 true）。
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

実装を変えたら `docs/` を更新すること（中粒度・Mermaid 図・日本語）。
- `docs/dracomancer_system.md`: システム仕様、モード、トピックフロー、起動引数
