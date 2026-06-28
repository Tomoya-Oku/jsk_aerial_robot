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
- `teleop_mode` は `startup` / `teleoperation`（`teleop` は別名）。`teleoperation` では形状（腕写像）が有効。移動（操縦桿）は既定 OFF（`enable_position_control:=false`。`control_position.py` の FlightNav が `POS_VEL_MODE`＋`target_pos` 未設定で落下するため。README「既知の課題」参照）。実行中は `/dracomancer/teleop_mode` (`std_msgs/String`) で切り替える。

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
- `teleoperation`: Dracomancer 腕関節を DRAGON 関節へマッピングし、**候補姿勢のフィージビリティで変形可否を判定**する（移動は `control_position.py` が同時に担当）。マッピングは `~mapping_mode` で切替（`distal`=既定: 遠位腕関節（手首＋肘）を DRAGON 関節へ**絶対角で一致**（中立記録なし。既定対応: 手首屈曲→joint1_pitch、手首内外転→joint1_yaw、肘屈曲→joint2_yaw。`clamp(sign*scale*source)`、`distal_signs` 既定 -1、平行リスト `distal_source_joints`/`distal_target_joints`/`distal_signs`/`distal_scales` で設定。未対応関節は直前値で固定。旧名 `elbow_only` は後方互換エイリアス） / `joint_pairing`: 3屈曲関節→3 yaw・pitch=0で平面保持 / `geometric`: 腕FK→面内yaw・面外pitch分解）。詳細は [README.md](README.md) の関節マッピング節。
- **予測フィージビリティ・ゲート**: 候補 DRAGON 形状を `shape_feasibility_node`（C++、`dragon/full_vectoring_robot_model` を pluginlib で読み込み、`ns=dragon` で起動）の `check_shape` サービスに渡し `fc_f_min`/`fc_t_min` を予測。force・torque 両方が下限しきい値以上なら変形（採用・記憶）、未満／サービス失敗なら直前可行姿勢で停止。**ただし現状は既定 OFF（`enable_feasibility_gate:=false`）**：フルベクタリング DRAGON は単発 `updateRobotModel` だとジンバルが水平のままで予測 fc がどの姿勢でも ≈0 になり、全変形を却下して形状が凍結するため。再有効化にはコントローラのベクタリング最適化済み fc を予測器側で再現する必要がある（README「既知の課題」参照）。下限しきい値は `*_volume_radius_threshold`（`[hard_min, min]` の `hard_min`）を購読、未受信時は `force_radius_threshold`/`torque_radius_threshold` パラメータ。
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

実装を変えたら関連ドキュメントを更新すること（中粒度・Mermaid 図・日本語）。

- `README.md`: システム仕様、モード、トピックフロー、関節マッピング、安全機構、起動引数（旧 `docs/dracomancer_system.md` を統合）
- `docs/`: 個別トピックの補足（例: `fc_threshold_calibration.md`）

## 研究コンテキスト（Dracomancer 固有）

> ルートの研究インストラクション（`AGENTS.md`）から Dracomancer デバイス固有の記述を移設したもの。
> 研究分野・研究背景・研究目的・作業規約（注意点）はルート `AGENTS.md` を参照。
> 以下の各節（方法・アイデア／進捗／これからやりたいこと）を変更する場合は、事前にユーザーへ確認すること。

### 使用機材

- 親機PC: Ubuntu22.04・ROS1 (noetic)
- 子機PC (Dracomancer背中部に搭載): Khadas VIM4
- マイコン: Spinal
- 3DCAD: SolidWorks2024
- Dracomancer
  - サーボモータ: DYNAMIXEL XL430-W250-T * 7
  - リンク: PLA (Bambu Lab)

### 方法・アイデア

- 立ち上げ時
  - DRAGONはテイクオフ時、円形形状でテイクオフする必要がある
  - この際に人間の腕形状からマッピングを行うことはできないため、どのようなシステムで行うかが未定
- 精密動作モード
  - 人間の腕形状をできるだけそのままDRAGONにマッピングする
  - 不可能姿勢への対処
    - Force/Torque Volumeの使用
    - 不可能姿勢をサーボモータによるトルク発揮で制限
- 広域移動モード
  - デバイスのグリップ部分にあるジョイスティックによる操作
    - デバイスの背中部分にIMUが搭載されており、操縦者の姿勢角度によってジョイスティックの移動方向を相対変化させたい
  - 人間の腕の動作を意図に変換（アイデア段階）
    - 機械学習の使用？
- 力覚提示機能
  - 人間の上肢7関節に対応するサーボモータを搭載し、力覚提示を可能にする
  - 力覚提示機能以外にサーボモータの使用によって可能になることを模索したい
    - 関節角を取得するだけなら、エンコーダ・モーションキャプチャだけで良いのでは？という反論

### 進捗

- 上肢外骨格デバイスハードウェアのプロトタイプ（Mk-I）
  - 背中・腰にかけてデバイス固定のためのベースウェアを装着
  - ベースウェアの背中部分を中心にデバイスを固定
  - 人間の関節数と同じ7関節にサーボモータを搭載

### これからやりたいこと

- システムの確立→システム図と数学的な手法の確立
  - 立ち上げ時
  - 精密動作モード
    - 適切なマッピング（複数比較）
    - 不可能姿勢の制限
  - 広域移動モード
    - ジョイスティック+IMUによる相対操縦（先行研究を参考にしたい）
    - 時間があれば機械学習による意図変換
  - 力覚提示機能（特に重要）
    - 疑問点
      - 数学的に何自由度までを発揮できるか
- 上肢外骨格デバイスハードウェアの改良版作成（Mk-II）
  - リンク長の調整→インデックスプランジャの使用
  - デバイスの干渉部などの詳細部の修正
  - サーボモータのグレードアップ
