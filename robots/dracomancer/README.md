# Dracomancer

このドキュメントは、Dracomancer の現在の実装に基づくシステム仕様をまとめたものです。
Dracomancer は、オペレータの腕に装着する上肢外骨格型遠隔操作デバイスであり、現状では主に DRAGON の移動指令と形状指令を生成します。

> 現状の実装メモ: 主な処理は `scripts/` 配下の Python ノードが担い、`src/shape_feasibility_node.cpp` は候補形状を DRAGON モデルで評価する実装済み C++ サービスです。それ以外の `src/` / `include/` の controller・model はプレースホルダです。力覚提示は安全スケーリングで抑制された形状入力から提示トルク相当量を計算しますが、Mk-I の XL430-W250-T では個々のサーボへ所望電流・所望トルクを入力できないため、Mk-I の既定実機出力はトルク ON/OFF です。互換サーボ向けの電流指令経路は残しています。

## 全体構成

Dracomancer は、デバイス自身の起動と DRAGON への遠隔操作変換を launch 単位で分けています。

```mermaid
flowchart TB
    subgraph BU["bringup.launch（Dracomancer 側）"]
        URDF["URDF / TF"]
        SB["servo_bridge または publish_fake_joint_states"]
        S2J["サーボ → 関節状態 変換"]
        FC["Spinal FC / IMU 接続<br/>（任意）"]
        WEB["Web Console<br/>（任意）"]
    end
    subgraph TO["teleoperation.launch（操作変換）"]
        CALIB["joystick 較正"]
        CP["位置指令 control_position.py"]
        AC["姿勢指令 control_orientation.py"]
        CJ["関節角指令 control_joint_angle.py"]
        SM["shape_safety監視<br/>volume_radius_monitor.py"]
    end
    subgraph HP["haptics.launch（力覚提示）"]
        HF["control_haptic_feedback.py"]
    end
    BU --> TO
    TO --> DRAGON["DRAGON"]
    TO --> HP
```

主なノード構成:

| ノード | 役割 | 主な入力 | 主な出力 |
| --- | --- | --- | --- |
| `calibrate_joystick.py` | 操縦桿の生値を較正して正規化 | `/joystick/raw` | `/dracomancer/joystick/calibrated` |
| `convert_servo_to_joint_states.py` | `bringup.launch` でサーボtickをラジアンの関節状態へ変換 | `/dracomancer/servo/states` | `/dracomancer/joint_states` |
| `control_position.py` | 操縦桿とIMUから DRAGON の速度指令を生成 | joystick, IMU, flight_state | `/dragon/uav/nav` |
| `control_orientation.py` | 操縦桿から姿勢指令を生成 | joystick, flight_state | `/dragon/final_target_baselink_rpy` |
| `control_joint_angle.py` | 腕関節から DRAGON の形状指令を生成（候補姿勢のフィージビリティで変形可否を判定） | joint_states, shape_feasibility, threshold, flight_state | `/dragon/joints_ctrl`, `/dracomancer/shape_control_error`, `/dracomancer/candidate/joint_target`, `/dracomancer/joint_map/switch_ratio` |
| `shape_feasibility_node`（C++） | 候補リンク角の force/torque volume 半径と結合ホバーwrench可行性を DRAGON モデルで予測するサービス | candidate joints | `~check_shape`（fc_f_min, fc_t_min, stability_ok） |
| `volume_radius_monitor.py` | しきい値 pub/sub・ライブ安全スケール算出（通常は teleoperation.launch で起動。fc 内接半径は再 pub しない） | fc inradius, threshold cmd | `*_volume_radius_threshold`, `/dracomancer/dragon_shape_safety_scale` |
| `servo_labels.py` | サーボtickを中心値基準のdeg/radへ変換、および関節スイッチング配分比をRVizテキスト表示 | `/dracomancer/servo/states`, `/dracomancer/joint_map/switch_ratio` | `/dracomancer/servo_angle_markers` |
| `control_haptic_feedback.py` | 抑制された形状入力から提示トルク相当量を計算し、既定でサーボのトルク ON/OFF を出力し、互換サーボ向けには電流指令も任意出力 | joint_states, shape_control_error, mode | `/dracomancer/haptic_torque`, `/servo/torque_enable`, `/servo/target_current` |
| `publish_fake_joint_states.py` | 実機なしで Dracomancer 関節状態を生成 | `/dracomancer/joint_cmd` | `/dracomancer/joint_states` |

デバイス側ハードウェア:

- 腕関節サーボは ID 0-6 の 7 個（肩・上腕・肘・手首）。
- サーボ状態は Spinal / FC 経由で `/dracomancer/servo/states` として扱います。
- 背中部 IMU は `/dracomancer/imu` として、操作者相対の移動方向補正に使います。
- M5Stack / 操縦桿は `/joystick/raw` の入力元です。

## テレオペレーションのデータフロー

Dracomancer から DRAGON への指令は、位置系統と形状系統の2つに分かれます。

```mermaid
flowchart TB
    subgraph POSE["位置系統（操縦桿 → 機体移動）"]
        A1["M5 / 操縦桿"] --> A2["/joystick/raw"]
        A2 --> A3["calibrate_joystick.py"]
        A3 --> A4["/dracomancer/joystick/calibrated"]
        A4 --> A5["control_position.py"]
        A5 --> A6["/dragon/uav/nav"]
    end
    subgraph JOINT["形状系統（腕の動き → 機体関節）"]
        B1["Dracomancer サーボ状態"] --> B2["/dracomancer/servo/states"]
        B2 --> B3["convert_servo_to_joint_states.py<br/>（bringup.launch）"]
        B3 --> B4["/dracomancer/joint_states"]
        B4 --> B5["control_joint_angle.py"]
        B5 --> B6["/dragon/joints_ctrl"]
        B5 --> B7["/dracomancer/shape_control_error"]
        B7 --> B8["control_haptic_feedback.py"]
        B4 --> B8
        B8 --> B9["/dracomancer/haptic_torque"]
        B5 --> B10["/dracomancer/candidate/joint_target"]
        B5 --> B11["/dracomancer/joint_map/switch_ratio"]
    end
```

主なトピック:

| トピック | 型 | 説明 |
| --- | --- | --- |
| `/joystick/raw` | `std_msgs/Int16MultiArray` | 操縦桿の生値 |
| `/dracomancer/joystick/calibrated` | `std_msgs/Float32MultiArray` | 較正済み軸値 |
| `/dracomancer/servo/states` | `spinal/ServoStates` | 腕サーボの状態 |
| `/dracomancer/servo_angle_markers` | `visualization_msgs/MarkerArray` | RViz用のサーボtick基準角度ラベル |
| `/dracomancer/imu` | `spinal/Imu` | 操作者姿勢のIMU quaternion |
| `/dracomancer/joint_states` | `sensor_msgs/JointState` | Dracomancer の腕関節角 |
| `/dragon/uav/nav` | `aerial_robot_msgs/FlightNav` | DRAGON の速度指令 |
| `/dragon/joints_ctrl` | `sensor_msgs/JointState` | DRAGON の形状指令 |
| `/dracomancer/dragon_shape_safety_scale` | `std_msgs/Float64` | ライブ安全スケール（`volume_radius_monitor.py` が pub、web UI が購読。情報提供用） |
| `/dracomancer/shape_control_error` | `std_msgs/Float64MultiArray` | 力覚提示用の形状抑制量 `q_des - q_tar` |
| `/dracomancer/haptic_torque` | `sensor_msgs/JointState` | 安全スケーリングで抑制された入力差から計算した Dracomancer 7関節の提示トルク |
| `/dracomancer/candidate/joint_target` | `sensor_msgs/JointState` | フィージビリティ・ゲート前の候補 DRAGON 関節角（`mapped_target()` の出力そのもの。teleoperationモードのみ） |
| `/dracomancer/joint_map/switch_ratio` | `std_msgs/Float64MultiArray` | 手首/肘のロール配分量 `[r1, rho1, c1_pitch, c1_yaw, r2, rho2, c2_pitch, c2_yaw]`（joint1=手首, joint2=肘。`r_i` は飽和後の配分入力、`rho_i` はyaw側寄与の目安 [0,1]、`c_i_pitch/c_i_yaw` は実際に使われるpitch/yaw重み） |

## 操作モード

`teleoperation.launch` は `teleop_mode` で、保持状態（`false`）と遠隔操作有効（`true`）を切り替えます。起動時の既定は `false` です。実行中は `/dracomancer/teleop_mode` に `std_msgs/Bool` を送ることで切り替えられます。`true` では、**腕形状による精密動作**が有効になります（以前の `wide` / `precision` の2モードを統合）。操縦桿による広域移動は既定で OFF（`enable_position_control:=false`）です（後述「既知の課題」の `control_position` FlightNav モード問題のため）。

```mermaid
stateDiagram-v2
    state "false" as off
    state "true" as on
    [*] --> off
    off --> on: 操作開始（移動＋形状）
    on --> off: 安全姿勢へ復帰
    note right of on
      腕形状写像を提供
      操縦桿移動はenable_position_control=true時のみ
    end note
```

| モード | 移動指令 `/dragon/uav/nav` | 形状指令 `/dragon/joints_ctrl` | 用途 |
| --- | --- | --- | --- |
| `false` | 送信しない | `startup_pose` を保持 | 離陸前後に DRAGON を通常姿勢へ保つ |
| `true` | `enable_position_control=true` のときジョイスティック + IMU 相対移動を送信 | Dracomancer 腕関節を DRAGON へマッピング | 接触作業。既定では形状変形とlink4固定を行う |

切り替え例:

```bash
rosrun dracomancer set_teleop_mode.py true
rosrun dracomancer set_teleop_mode.py false
```

### 位置系統

`control_position.py` は、`teleop_mode=true` のときだけ較正済み操縦桿入力を `FlightNav` の速度指令へ変換します。DRAGON がホバリング状態（`flight_state >= 4`）になってから送信し、ホバ直後は `wait_after_hover`（既定 3 秒）待機します。

```mermaid
flowchart LR
    JOY["calibrated axes"] --> DZ["deadzone<br/>0.05"]
    DZ --> VEL["body velocity<br/>xy_vel=0.3, z_vel=0.2"]
    VEL --> ROT["direction_mode による向き補正"]
    IMU["/dracomancer/imu"] -.-> ROT
    ROT --> NAV["/dragon/uav/nav<br/>WORLD_FRAME / POS_VEL_MODE"]
```

ジョイスティック仕様:

- `teleoperation.launch` の joystick 較正は既定で `axis_indices=[0, 1]` の2軸のみです。
- `axis_x=0` を前後、`axis_y=1` を左右、`axis_z=2` を上下として扱います。
- 既定の較正済みメッセージには3軸目が無いため、Z方向入力は0になります。Z軸を使う場合は、操縦桿側と較正設定を3軸に拡張してください。
- `xy_vel=0.3`、`z_vel=0.2` が既定の最大速度スケールです。
- `pos_limit=true` のとき、`/dragon/mocap/pose` が部屋範囲外または境界にある場合は、さらに外側へ向かう速度成分を0にします。

### IMU 相対移動

`direction_mode` により、操縦桿の移動方向を操作者の向きに対する相対方向へ変換します。spinal は磁気を使わない場合 yaw がドリフトしやすいため、起動時またはホバ開始時の中立姿勢からの相対回転を使います。

```mermaid
flowchart TB
    Q["q_spinal（board→world）"] --> WO["q_wo = q_spinal ⊗ q_mount"]
    MNT["imu_mount_rpy"] --> WO
    WO --> CAP["中立キャプチャ"]
    CAP --> REL["相対回転 / 相対 yaw,pitch"]
    REL --> CMD["body velocity を world velocity へ変換"]
```

| `direction_mode` | 挙動 | 用途 |
| --- | --- | --- |
| `none` | 操縦桿をそのまま world 速度として使う | IMUを使わない |
| `yaw` | 中立からの相対ヨーで水平移動方向を回転 | 既定。最も扱いやすい想定 |
| `yaw_pitch` | 相対ヨーと相対ピッチを反映 | 前傾も移動方向へ反映したい場合 |
| `full` | 相対3D回転をそのまま反映 | 全身姿勢を強く反映する実験用 |

中立は以下で取り直します。

- 起動後、最初に有効なIMUを受け取った時
- `recapture_neutral_on_hover=true` の状態でホバリングへ遷移した時
- `~recapture_neutral` に `std_msgs/Empty` を送った時

手動取り直し例:

```bash
rostopic pub -1 /dracomancer_control_position/recapture_neutral std_msgs/Empty "{}"
```

移動対象は `nav_target` で切り替えます。

| `nav_target` | `FlightNav.target` | 意味 |
| --- | --- | --- |
| `cog` | `FlightNav.COG` | 重心基準で移動（既定） |
| `baselink` | `FlightNav.BASELINK` | ベースリンク基準で移動 |

## 立ち上げ・精密・広域移動の形状制御

`control_joint_angle.py` は全モードで `/dragon/joints_ctrl` を担当します。モードごとの目標形状は以下です。

| モード | 目標形状 |
| --- | --- |
| `false` | `startup_pose`。既定 `[0, pi/2, 0, pi/2, 0, pi/2]` |
| `true` | Dracomancer 腕関節からのマッピング結果 |

`startup_pose` は DRAGON の通常姿勢へ戻す `transformation_demo.py _reset:=1` と同じ考え方で、離陸時に人間の腕形状へ直接マッピングできない問題を避けるための保持姿勢です。実機で離陸前の関節指令が反映されるかは、DRAGON 側の preflight joint control 設定に依存します。

## 関節マッピング

Dracomancer の腕関節を、DRAGON を1本の直列アームとみなして対応付けます（DRAGON は `joint*_pitch=0` で水平面に整列し、`joint*_yaw` が面内のセルペンタイン形状を作る）。実装は [control_joint_angle.py](scripts/control/control_joint_angle.py) にあり、`teleoperation.launch` の引数で方式・基準姿勢・安全ゲート挙動を切り替えます。

| `mapping_mode` | 概要 |
| --- | --- |
| `joint_pairing` | 3つの屈曲関節を DRAGON の3つの yaw に1:1対応、pitch は 0 固定で平面保持 |
| `geometric` | 腕の順運動学からリンク方向ベクトルを求め、面内(yaw)/面外(pitch)成分に分解 |
| `distal`（**既定**） | 遠位腕関節（手首・肘）を DRAGON 関節へ絶対角で一致させる（pitch/yaw配分に使う上腕・前腕ロールのみエンゲージ時中立からの差分。手首屈曲→上腕ロール+前腕ロール角に応じてjoint1_pitch/joint1_yawへ配分、肘屈曲→上腕ロール角に応じてjoint2_pitch/joint2_yawへ配分、肩屈曲→joint3_pitch、肩内外転→joint3_yaw。上腕ロール+前腕ロールの差分和はbaselink rollへ加算）。旧名 `elbow_only` も後方互換で可 |

### joint_pairing（中期方式）

屈曲関節（肩・肘・手首、すべて −X 軸）は同軸なので、これだけ動かすと腕は1平面内で曲がります。これを DRAGON の yaw に流し、pitch を定数に固定することで「**腕の面内曲げ＝DRAGON の面内形状**」を構造的に保証します。

| DRAGON 関節 | Dracomancer 関節 | sign | offset | 面 |
| --- | --- | --- | --- | --- |
| `joint1_pitch` | （定数） | – | `0` | 面外→0 |
| `joint1_yaw` | `wrist_flexion_extension_joint` | -1 | `0` または `pi/2` | 面内 |
| `joint2_pitch` | （定数） | – | `0` | 面外→0 |
| `joint2_yaw` | `elbow_flexion_extension_joint` | -1 | `0` または `pi/2` | 面内 |
| `joint3_pitch` | （定数） | – | `0` | 面外→0 |
| `joint3_yaw` | `shoulder_flexion_extension_joint` | -1 | `0` または `pi/2` | 面内 |

```text
source 名が空 -> その関節は定数（= offset）。pitch を 0 に保つために使用。
それ以外      -> mapped = offset[i] + sign[i]*(joint_pairing_scale*scale[i])*(source - neutral)
target        = clamp(mapped, -joint_limit, joint_limit)
```

- yaw の sign は蛇の曲がり向き。DRAGON が逆向きに曲がる場合は3つ揃えて反転（`geom_yaw_sign` も合わせる）。
- `mapping_reference=straight` は yaw の offset を 0 とする方式（DRAGON を真っ直ぐ伸ばす基準）。腕角度を DRAGON yaw へ直接入れるため、円形姿勢 `pi/2` から大きく離れた候補が出やすい。
- `mapping_reference=circular`（既定）は `startup_pose=[0, pi/2, 0, pi/2, 0, pi/2]` を offset に使い、円形姿勢からの相対変形にする。テイクオフ円形からそのまま変形でき、形状制御試験の推奨設定。
- ※ `mapping_reference` は全 `mapping_mode` 共通。旧名 `joint_pairing_reference` と旧値 `zero`/`startup` も後方互換で使用可（それぞれ `straight`/`circular` に対応）。
- `joint_pairing_scale` は全関節の写像ゲインに掛かる一括係数。安全ゲートを残す試験では `0.2〜0.4` 程度から始める。
- `capture_neutral_on_first_msg=true` にすると、最初に受け取った Dracomancer 関節角を `neutral` として記憶し、以後はそこからの差分を使う。
- `joint_limit=pi/2` が既定。
- `distal` では捻り（`upper_arm_external_internal_rotation_joint` と `wrist_supination_joint`）を最初の受信値・ホバー開始時・link4アンカー再取得時の中立値からの差分として合算し、baselink roll に加算する（`enable_baselink_roll_mapping`）。

推奨する形状制御試験設定:

```bash
roslaunch dracomancer teleoperation.launch \
  mapping_mode:=joint_pairing \
  teleop_mode:=true \
  mapping_reference:=circular \
  capture_neutral_on_first_msg:=true \
  joint_pairing_scale:=0.3
```

### geometric（長期方式・引数で選択）

`urdf/dracomancer.urdf` のリンク鎖で腕を順運動学し、上腕・前腕・手のベクトルを求め、隣接リンク間の相対回転を平面法線（既定 `geom_plane_normal=[1,0,0]`）まわりの **azimuth（→yaw）/ elevation（→pitch）** に分解します。中立姿勢を基準に差分を取るため、中立で全関節 0、屈曲を曲げると対応する1関節のみ yaw が動きます（数値検証済み）。

```text
FK -> 上腕/前腕/手の方向ベクトル
各関節(肩→joint3, 肘→joint2, 手首→joint1)で
  yaw   = geom_yaw_sign  * geom_yaw_scale  * (相対azimuth   - 中立azimuth)
  pitch = geom_pitch_sign* geom_pitch_scale* (相対elevation - 中立elevation)
```

- 主用途の屈曲ベース面内整形はクリーン。外転・捻りなど面外DOFは、リンク構造オフセットの影響で遠位関節に pitch/yaw 結合が出ることがある（既知の限界、研究比較用）。
- パラメータ: `geom_chain`, `geom_plane_normal`, `geom_yaw_sign/scale`, `geom_pitch_sign/scale`。

### distal（遠位腕関節の絶対一致・**既定**）

> 旧名 `elbow_only`（後方互換のエイリアスとして引き続き使用可）。肘1自由度だった写像を、手首・肩を加えた**腕関節の絶対角一致**に一般化したものです。

人間の腕の各関節を、DRAGON の各関節に**絶対角で一致**させます（人間が 90° なら DRAGON も 90°）。ただし **pitch/yaw 配分に使う上腕・前腕ロールのみ**、`teleop_mode=true` への切替時（およびホバー開始時・`~recapture_anchor` 受信時）にその瞬間の値を中立としてキャプチャし、以後は中立からの差分を配分入力に使います（`capture_roll_neutral` 既定 `true`。ロールサーボの組立零点ズレで配分が飽和端に張り付くのを防ぐ）。既定の対応は以下の5つ：

| 人間の腕関節（source） | DRAGON 関節（target） | 意味 |
| --- | --- | --- |
| `wrist_flexion_extension_joint` | `joint1_pitch` / `joint1_yaw` | 手首の屈曲/伸展 → 上腕ロール+前腕ロールが0度付近ではpitch、総和が大きいとyaw |
| `wrist_abduction_adduction_joint` | `joint1_yaw` | 手のひら平面と平行な手首回転（内外転）→ yaw（手首ロール切替ON時は手首屈曲と同じ2Dベクトルとして回転） |
| `elbow_flexion_extension_joint` | `joint2_pitch` / `joint2_yaw` | 肘の屈曲 → 上腕ロール0度付近ではpitch、ロールが大きいとyaw |
| `shoulder_flexion_extension_joint` | `joint3_pitch` | 肩の屈曲/伸展 → pitch（オフセットなし） |
| `shoulder_abduction_adduction_joint` | `joint3_yaw` | 肩の内外転 → yaw |

対応リストに無い関節は**直前に指令した値のまま保持され、一切変化しません**。ただし既定では `enable_wrist_roll_switching=true` と `enable_elbow_roll_switching=true` により、`wrist_flexion_extension_joint` は `joint1_pitch` / `joint1_yaw` へ、`elbow_flexion_extension_joint` は `joint2_pitch` / `joint2_yaw` へ、`distal_target_joints` 適用後に再配分されます。

```text
target_joint[k] = clamp(sign[k] * scale[k] * source_angle[k] + offset[k],  -joint_limit, joint_limit)
  ※ pitch/yaw配分のロール入力のみ r = (ロール角 − エンゲージ時中立) − ロールオフセット（capture_roll_neutral 既定true）
未対応の関節     = 直前の可行値（last_feasible_target、変化させない）
```

- `capture_neutral_on_first_msg` は distal では不使用（distal は配分ロールのみエンゲージ時キャプチャ、屈曲・内外転系は中立記録なしの絶対一致）。
- `distal_signs` 既定は `[1.0, -1.0, 1.0, 1.0, -1.0]`（手首内外転・肩内外転が -1、他は +1。各関節ごとに、DRAGON が逆向きに動く場合に反転）。`distal_scales` 既定 `1.0` で 1:1 角度一致。
- `distal_offsets` 既定は `[0, 0, 0, 0, 0]`：joint3_pitch は以前の `+π/2` 肩屈曲オフセットを使わず、`sign=+1` の絶対角一致で扱う（屈曲・内外転系サーボの零点ズレは組立較正または `distal_offsets` で対処する）。肩内外転の向きは未検証なので sim で要確認。
- **手首のロール総和配分**（`enable_wrist_roll_switching` 既定 ON）: `wrist_roll_joints`（既定: `upper_arm_external_internal_rotation_joint` + `wrist_supination_joint`）の各ロール角から**エンゲージ時中立**と `wrist_roll_offsets`（既定 `[0, 0]`）を先に引き、各差分を ±90° で飽和する。その和を `alpha = theta + phi` として扱い、手首屈曲と手首内外転の2Dベクトルを `cos(alpha):sin(alpha)` で `joint1_pitch` / `joint1_yaw` へ回転配分する。`alpha > 90°` で `joint1_pitch` 側の符号が反転する挙動は維持する。
- **肘の上腕ロール配分**（`enable_elbow_roll_switching` 既定 ON）: `upper_arm_external_internal_rotation_joint` から**エンゲージ時中立**と `elbow_roll_offset`（既定0）を先に引き、その差分を ±90° で飽和する（飽和後の値を `r` とする）。肘屈曲を `joint2_pitch:joint2_yaw = r:(π/2 - |r|)` の線形比で配分する。上腕ロールの符号は `joint2_pitch` 側に反映され（ロール方向で曲げ面が反転）、`joint2_yaw` 側の重みは偶関数なので中立ロールをまたいでも連続に変化する。`joint2_yaw` の符号は `elbow_yaw_sign`（既定 `1.0`）で必要に応じて反転できる。
- 手首・肘それぞれの配分比 `rho_i` とその入力・重みは `/dracomancer/joint_map/switch_ratio` に毎周期publishされ、比較・解析用に記録できる（`scripts/shape_task/task_recorder.py` は `r1,rho1,c1_pitch,c1_yaw,r2,rho2,c2_pitch,c2_yaw` 列としてCSVに記録）。
- **link4 アンカー（`enable_link4_anchor` 既定 ON）**: 関節を曲げても DRAGON の **link4（腕先端）位置をワールドにおおよそ固定**するため、ホバー開始時に link4 位置を基準化し、`joints_ctrl` と同じ目標関節角からCOG位置（`uav/nav` POS_MODE）を逆算する。既定の `link4_anchor_mode:=position_only` ではCOG位置だけを補償し、link4 yawは固定しない。`link4_anchor_mode:=position_yaw` ではCOG位置に加えてCOG yaw目標（`uav/nav` の `target_yaw`）でlink4 yawも固定する（baselink姿勢補償は送らない）。`link4_anchor_mode:=full` ではCOG位置+baselink姿勢でlink4姿勢も補償できるが、姿勢failsafeに近づきやすいため明示指定時のみ使う。ON時も `enable_link4_anchor_body_safety` がCOG高度・水平リーシュ・full時の姿勢を検査し、`enable_link4_anchor_tracking_safety` がCOG/yaw/roll/pitch追従遅れを検査し、`enable_link4_anchor_joint_tracking_safety` がDRAGON関節追従遅れを検査する。危険なbody補償、追従遅れ、TF断時は関節/補償指令を保持する。詳細は [docs/link4_anchor.md](docs/link4_anchor.md)。**移動制御（`enable_position_control`）とは併用不可**（`uav/nav` が競合）。
- 未対応の関節は `mapping_reference` の straight/circular に関係なく動かない。
- 対応関係は平行リスト `distal_source_joints` / `distal_target_joints` / `distal_signs` / `distal_scales`（同じ長さ）で自由に変更可。

```bash
roslaunch dracomancer teleoperation.launch \
  teleop_mode:=true
```

3方式とも出力はフィージビリティ・ゲートを通って DRAGON へ送られます。既定では `enable_feasibility_gate:=true` で、Force/Torque Volume radius の下限、または同じ推力配分によるホバーforce/ゼロtorqueの同時可行性を満たさない候補姿勢は `feasibility_gate_mode` に従って抑制されます。予測器が過度に保守的で変形が凍結する場合は、起動時に `enable_feasibility_gate:=false` を指定して候補姿勢をそのまま送れます。

サーボIDと Dracomancer 関節名:

| サーボID | 関節名 |
| --- | --- |
| 0 | `shoulder_abduction_adduction_joint` |
| 1 | `shoulder_flexion_extension_joint` |
| 2 | `upper_arm_external_internal_rotation_joint` |
| 3 | `elbow_flexion_extension_joint` |
| 4 | `wrist_supination_joint` |
| 5 | `wrist_flexion_extension_joint` |
| 6 | `wrist_abduction_adduction_joint`（`distal` では `joint1_yaw` 入力。手首ロール切替ON時は手首屈曲と同じ2Dベクトルとして回転） |

サーボtickから関節角への変換:

```text
rad = (tick - 2048) * 2*pi / 4096 + offset
```

## 形状安全機構

形状安全は **予測フィージビリティ・ゲート**（`teleop_mode=true` の主機構）と **ライブ監視**（情報提供）の2層です。

### 1. 予測フィージビリティ・ゲート（`teleop_mode=true`）

> **既定で有効（`enable_feasibility_gate:=true`）です。** 候補姿勢の予測 `fc_f_min` / `fc_t_min` がしきい値を下回る場合、または1つの共通推力配分でホバーforceとゼロtorqueを同時生成できない場合、危険姿勢回避として候補を抑制します。フルベクタリング DRAGON のモデル/設定によって予測器が過度に保守的になり変形が凍結する場合は、検証用に `enable_feasibility_gate:=false` で無効化できます。

腕関節を DRAGON 形状にマッピングした**候補姿勢**を、実際に送る前に評価します。

```mermaid
flowchart TD
    A["腕関節 → 候補 DRAGON 形状"] --> B["shape_feasibility サービスで予測<br/>fc_f_min, fc_t_min, stability_ok（DRAGONモデル）"]
    B --> C{"fc_f_min >= force_hard<br/>かつ fc_t_min >= torque_hard<br/>かつ stability_ok ?"}
    C -->|yes| D["変形可：候補を採用し記憶<br/>last_feasible_target = 候補"]
    C -->|no / サービス失敗| E{"feasibility_gate_mode"}
    E -->|hold| H["直前の可行姿勢を保持<br/>min以上かつstability_okで再開"]
    E -->|step_search| I["直前可行姿勢から候補方向へ<br/>小刻みに探索"]
    E -->|soft_scale| J["fc余裕に応じて候補方向へ進める<br/>stability_ok=falseなら保持"]
    D --> F["max_step で1周期の変化量を制限して送信"]
    H --> F
    I --> F
    J --> F
```

- **判定基準**：force・torque **両方**の予測半径が `hard_min` 以上であり、かつ `stability_ok=true`（同じ推力配分でホバーforceとゼロtorqueを生成可能）の場合だけ変形可。`hold` モードで一度不可になった場合は、両方が `min` 以上へ復帰し、結合判定も通るまで直前の可行姿勢を保持する（ヒステリシス）。
- **NG時**：`feasibility_gate_mode` に応じて、保持・小ステップ探索・縮小移動のいずれかを行う。
- 予測はサービス `shape_feasibility/check_shape` が `dragon/full_vectoring_robot_model` プラグインで計算します。既定の `shape_feasibility_prediction_mode=allocation` では、候補形状に対して静的ホバーallocationを行い、割当後のgimbal roll角と利用可能なbaselink姿勢feedbackを使って `calcFeasibleControlFxyDists()` / `calcFeasibleControlTDists()` を再評価します。さらに、同じgimbal・姿勢状態の評価用モデルで基底`RobotModel::stabilityCheck()`を実行し、同じ静的推力配分によるforce/torque残差と推力上下限を`stability_ok`として評価します。`controller` は最新の `/dragon/gimbals_ctrl` のgimbal roll/pitch指令を使う旧近似、`optimized_gimbal` はDRAGONモデルのジンバル処理済み状態だけを使う形状中心の予測です。
- 最適化が毎回走るため、評価は `feasibility_rate`（既定 10Hz）にスロットルされます。CPU負荷が高い場合はlaunch引数で下げられます。

> `shape_feasibility_node` は DRAGON の namespace（`ns=dragon`）で起動し、モデルが `/dragon/robot_description` と機体パラメータを読みます。**DRAGON が起動している必要があります。**

予測fcに使う要素:

| 要素 | 使うモード | 入力元 | 役割 |
| --- | --- | --- | --- |
| 候補DRAGON関節角 | `model` / `optimized_gimbal` / `controller` / `allocation` | `control_joint_angle.py` から `check_shape` serviceへ渡す `name[]` / `position[]` | link姿勢・ロータ位置・基本的なFC余裕を決める主入力 |
| DRAGONモデル・機体パラメータ | 全モード | `/dragon/robot_description`、rotor rosparam、`dragon/full_vectoring_robot_model` | 質量、リンク、ロータ配置、gimbal制約を読む |
| モデル内gimbal処理状態 | `optimized_gimbal` | `updateRobotModel()` 後の `getGimbalProcessedJoint()` / `getRollLockedGimbal()` | 候補形状だけからgimbal lockとroll角を推定する従来予測 |
| 最新gimbal roll/pitch指令 | `controller` | `/dragon/gimbals_ctrl` | controllerが実際に使うgimbal状態へ近づける |
| static allocation | `allocation` | 候補形状、DRAGONモデル、ホバー重力補償レンチ | 候補形状に応じたgimbal rollを予測する |
| baselink姿勢feedback | `controller` / `allocation` | `/dragon/final_target_baselink_rpy`、または `/dragon/uav/baselink/odom` | torque fc計算の `cog_rot` 条件へ反映する |
| しきい値 | gate判定 | `/dracomancer/force_volume_radius_threshold`、`/dracomancer/torque_volume_radius_threshold` | 予測fcを採用/保持へ変換する判定基準 |
| 最終送信target | 診断用 | gate / rate limit / link4 safety後の `current_target` | `/dracomancer/target/fc_*_min` として実測fc比較用にpublish |

| `feasibility_gate_mode` | 挙動 | 用途 |
| --- | --- | --- |
| `hold`（既定） | 候補が `hard_min` 未満、または `stability_ok=false` なら `last_feasible_target` を保持し、両半径が `min` 以上かつ `stability_ok=true` へ回復してから再開 | 境界付近で採用/拒否が振動しにくい保守的な挙動 |
| `step_search` | `last_feasible_target` から候補方向へ `feasibility_step_fraction` だけ進めた姿勢を評価し、不可なら半分にして再試行 | 安全ゲートを残して少しずつ変形させる試験 |
| `soft_scale` | 候補の fc と閾値の比から移動倍率を決める。`stability_ok=false`の場合は倍率0として直前可行姿勢を保持 | 力覚提示と組み合わせた「硬くなる」挙動の検討 |

`step_search` は `feasibility_min_step_fraction` 未満になるまで探索します。`soft_scale` の倍率は `min(fc_f/force_thr, fc_t/torque_thr, 1)` を基本とし、`feasibility_soft_min_scale` で下限を設定できます。ただし、結合判定が不可の場合は下限設定にかかわらず倍率0です。

### 2. ライブ監視（`volume_radius_monitor.py`、teleoperation.launch）

実機の**現在状態**の fc 内接半径からライブの安全スケールを算出して publish します（web UI 表示・記録用、ゲートとは独立）。通常は teleoperation 側で起動するため、`shape_safety` ログは `control_joint_angle.py` と同じ `teleoperation.launch` の端末に出ます。fc 内接半径そのものは **DRAGON の `/dragon/debug/fc_*_min` を直接購読**してください（Dracomancer では再 publish しません）。

### しきい値トピック（`volume_radius_monitor.py` が所有・pub/sub、ゲートも購読）

| トピック | 型 | 意味 |
| --- | --- | --- |
| `/dracomancer/force_volume_radius_threshold` | `std_msgs/Float64MultiArray` | 力のしきい値 `[hard_min, min]`（常時 publish） |
| `/dracomancer/torque_volume_radius_threshold` | `std_msgs/Float64MultiArray` | トルクのしきい値 `[hard_min, min]`（常時 publish） |
| `/dracomancer/force_volume_radius_threshold_cmd` | `std_msgs/Float64MultiArray` | 力のしきい値 `[hard_min, min]` を実行時に設定（subscribe） |
| `/dracomancer/torque_volume_radius_threshold_cmd` | `std_msgs/Float64MultiArray` | トルクのしきい値 `[hard_min, min]` を実行時に設定（subscribe） |

`control_joint_angle.py` はこれらの `[hard_min, min]` を予測ゲートに使います。`hard_min`（先頭）は拒否状態に入る下限、`min`（2番目）は `hold` モードで拒否状態から復帰する上限です。トピック未受信時は `force_radius_threshold`/`torque_radius_threshold`（hard側）と `force_radius_recover_threshold`/`torque_radius_recover_threshold`（min側）を使います。しきい値更新は `hard_min <= min` の場合のみ反映します。

### 送信ゲート（ホバリング以外では位置・姿勢・関節角操作を無効化）

| 条件 | 既定 | 挙動 |
| --- | --- | --- |
| `publish_joints_only_when_hovering` | `true` in `teleoperation.launch` | ホバリング前は `/dragon/joints_ctrl` を送らない |
| `publish_joints_before_device_ready` | `false` | `teleop_mode=true` では false なら Dracomancer 関節状態を受け取るまで送らない |

> `control_position.py`（`teleop_mode && hovering && !landing`）と `control_orientation.py`（`publish_only_when_hovering` 既定 true）も同様にホバリング時のみ出力します。

### 主なパラメータ

`control_joint_angle.py`（teleoperation.launch）:

| パラメータ | 既定 | 説明 |
| --- | --- | --- |
| `mapping_mode` | `distal` | 腕→DRAGON形状の写像方式（`elbow_only` は `distal` の別名） |
| `mapping_reference` | `circular` | 写像の offset 基準（全モード共通）。`straight`=0rad基準 / `circular`=円形姿勢基準。旧名 `joint_pairing_reference`、旧値 `zero`/`startup` も可 |
| `joint_pairing_scale` | `1.0` | `joint_pairing` の一括写像ゲイン |
| `distal_source_joints` | `[wrist_flexion_extension_joint, wrist_abduction_adduction_joint, elbow_flexion_extension_joint, shoulder_flexion_extension_joint, shoulder_abduction_adduction_joint]` | `distal` の入力（人間の腕）関節リスト |
| `distal_target_joints` | `[joint1_pitch, joint1_yaw, joint2_yaw, joint3_pitch, joint3_yaw]` | `distal` の出力 DRAGON 関節リスト（source と同順）。肘はこの基本写像後、`enable_elbow_roll_switching` により `joint2_pitch` / `joint2_yaw` へ再配分 |
| `distal_signs` | `[1.0, 1.0, 1.0, 1.0, -1.0]` | `distal` の各符号（逆向きに動く関節を反転） |
| `distal_scales` | `[1.0, 1.0, 1.0, 1.0, 1.0]` | `distal` の各ゲイン（`1.0` で 1:1 角度一致） |
| `distal_offsets` | `[0, 0, 0, 0, 0]` | `distal` の各加算オフセット[rad]（`sign*scale*source + offset`）。joint3_pitch は90degオフセットなし |
| `capture_roll_neutral` | `true` | 配分ロール（上腕・前腕）の中立を`teleop_mode=true`切替・ホバー開始・`~recapture_anchor` でキャプチャし、差分を配分入力に使う（`false` でロールも絶対角） |
| `enable_wrist_roll_switching` | `true` | ロール総和に応じて手首屈曲を `joint1_pitch` / `joint1_yaw` へ配分 |
| `wrist_roll_joints` | `[upper_arm_external_internal_rotation_joint, wrist_supination_joint]` | 手首のpitch/yaw配分に使うロール関節。各基準差分を ±90° で飽和してから足す |
| `wrist_roll_offsets` | `[0.0, 0.0]` | `wrist_roll_joints` と同順の基準角 [rad]。各ロールは基準差分を作ってから ±90° で飽和 |
| `wrist_pitch_sign` / `wrist_yaw_sign` | `1.0` / `1.0` | 手首屈曲を `joint1_pitch` / `joint1_yaw` へ入れる符号 |
| `wrist_pitch_scale` / `wrist_yaw_scale` | `1.0` / `1.0` | 手首屈曲を `joint1_pitch` / `joint1_yaw` へ入れるゲイン |
| `wrist_yaw_source_sign` / `wrist_yaw_source_scale` | `1.0` / `1.0` | 手首内外転を手首ロール切替の yaw 初期成分へ入れる符号・ゲイン |
| `enable_elbow_roll_switching` | `true` | 上腕ロール角に応じて肘屈曲を `joint2_pitch` / `joint2_yaw` へ配分 |
| `elbow_roll_offset` | `0.0` | 上腕ロールの基準角 [rad]。基準差分を作ってから ±90° で飽和 |
| `elbow_pitch_sign` / `elbow_yaw_sign` | `1.0` / `1.0` | 肘屈曲を `joint2_pitch` / `joint2_yaw` へ入れる符号 |
| `elbow_pitch_scale` / `elbow_yaw_scale` | `1.0` / `1.0` | 肘屈曲を `joint2_pitch` / `joint2_yaw` へ入れるゲイン |
| `enable_link4_anchor` | `true` | `distal` 時に link4 位置をワールド固定するためCOG位置を補償。`enable_position_control` とは併用不可 |
| `link4_anchor_mode` | `position_only` | `position_only`: link4位置だけ固定 / `position_yaw`: COG位置+COG yaw目標でlink4位置とyawを固定 / `full`: COG位置+baselink姿勢でlink4姿勢も補償 |
| `link4_anchor_offset_x` | `0.0` | 固定点の link4 x軸方向オフセット [m]。`0.474`（リンク長）で尾端を固定 |
| `publish_link4_anchor_baselink_motion` | `false` | `full` 時に `/dragon/target_rotation_motion` へbaselink姿勢を即時指令する。姿勢failsafeに近づきやすいため既定OFF |
| `enable_link4_anchor_body_step_scaling` | `true` | link4アンカーのCOG位置、またはfull時のbaselink姿勢の必要変化量が大きすぎる場合、関節ステップを自動縮小 |
| `link4_anchor_max_body_pos_rate` | `0.4` | body step scalingで許容するCOG位置目標の最大変化速度 [m/s] |
| `link4_anchor_max_body_rpy_rate` | `0.8` | body step scalingで許容する姿勢目標の最大変化速度 [rad/s]（`full`: baselink RPY / `position_yaw`: COG yaw） |
| `enable_link4_anchor_body_safety` | `true` | link4アンカー補償後のCOG高度・水平リーシュ、またはfull時のbaselink姿勢を検査し、危険なら関節/補償指令を保持 |
| `link4_anchor_max_abs_roll` / `link4_anchor_max_abs_pitch` | `0.6` / `0.6` | `full` 時のlink4アンカー補償で許容するbaselink roll/pitch絶対値 [rad] |
| `link4_anchor_min_cog_z` / `link4_anchor_max_cog_z` | `0.6` / `2.5` | link4アンカー補償で許容するCOG高度範囲 [m]。max `0.0` は上限チェック無効 |
| `link4_anchor_max_cog_xy_offset` | `0.9` | link4アンカー補償でホバー開始時COGから許容する水平距離 [m]。`0.0` 以下で無効 |
| `link4_anchor_max_abs_yaw_delta` | `2.094` | `position_yaw` のCOG yaw目標をホバー開始時COG yawからの差分で制限する上限 [rad]。`0.0` 以下で無効 |
| `enable_link4_anchor_tracking_safety` | `true` | 実COG/yaw/roll/pitchがlink4アンカー目標へ追従していない時に形状更新を保持 |
| `link4_anchor_max_cog_tracking_error` | `0.2` | COG位置目標と実COG位置の許容誤差 [m] |
| `link4_anchor_max_yaw_tracking_error` | `0.524` | `position_yaw` のCOG yaw目標と実yawの許容誤差 [rad] |
| `link4_anchor_max_tracking_roll` / `link4_anchor_max_tracking_pitch` | `0.262` / `0.262` | link4アンカー中に許容する実roll/pitch絶対値 [rad] |
| `enable_link4_anchor_joint_tracking_safety` | `true` | DRAGON実関節が`joints_ctrl`へ追従していない時に形状更新を保持 |
| `link4_anchor_max_joint_tracking_error` | `0.3` | DRAGON各関節の目標-実値差の許容値 [rad] |
| `link4_anchor_joint_tracking_timeout` | `0.5` | DRAGON関節状態を追従判定に使う最大経過時間 [s] |
| `enable_baselink_roll_mapping` | `true` | `distal` 時に上腕ロールと前腕ロールの中立値からの差分和をbaselink rollへ加算 |
| `baselink_roll_source_joints` | `[upper_arm_external_internal_rotation_joint, wrist_supination_joint]` | baselink roll に使う Dracomancer ロール関節 |
| `baselink_roll_signs` | `[-1.0, -1.0]` | baselink roll 差分の符号。回転方向が逆なら該当要素を反転 |
| `baselink_roll_scales` | `[1.0, 1.0]` | baselink roll 差分の各ゲイン |
| `baselink_roll_limit` | `π/2` | baselink roll へ加算する差分の絶対値上限 [rad] |
| `hover_flight_state` | `5` | DRAGON の HOVER_STATE。`/dragon/uav/nav` はHOVER以外では無視されるため、link4アンカーもこの状態でのみ有効 |
| `capture_neutral_on_first_msg` | `false` | 最初の Dracomancer 関節角を中立姿勢として記憶（`distal` では不使用。distal は配分ロールのみエンゲージ時キャプチャ） |
| `enable_feasibility_gate` | `true` | 予測ゲートの有効化。true でForce/Torque Volume radiusと結合ホバーwrench可行性に基づく危険姿勢回避を行う。false で候補をそのまま採用 |
| `feasibility_gate_mode` | `hold` | 不可行候補への対処。`hold` / `step_search` / `soft_scale` |
| `feasibility_step_fraction` | `0.25` | `step_search` の初期探索ステップ |
| `feasibility_min_step_fraction` | `0.03` | `step_search` の最小探索ステップ |
| `feasibility_soft_min_scale` | `0.0` | `soft_scale` の移動倍率下限 |
| `feasibility_service` | `/dragon/shape_feasibility/check_shape` | 予測サービス名 |
| `feasibility_rate` | `10.0` | 候補評価のスロットル周波数 [Hz] |
| `shape_feasibility_prediction_mode` | `allocation` | 予測fcの計算方法。`allocation` は候補形状に対して静的ホバーallocationを行い、割当後のgimbal rollでfcを再評価する。`controller` は最新gimbal roll/pitch指令・姿勢feedbackを使う旧近似、`optimized_gimbal` は従来の形状中心予測、`model` は `updateRobotModel()` 後の標準fc |
| `enable_target_fc_prediction` | `true` | gate/rate-limit/link4 safety後に実際へ送る最終targetの予測fcを `/dracomancer/target/fc_*_min` にpublish |
| `target_fc_prediction_rate` | `10.0` | 最終target予測fcの評価周波数 [Hz] |
| `force_radius_threshold` | `0.108990` | 力の下限しきい値（topic 未受信時のフォールバック） |
| `torque_radius_threshold` | `0.015400` | トルクの下限しきい値（同上） |
| `force_radius_recover_threshold` | `0.249220` | `hold` モードで拒否状態から復帰する力のしきい値（topic 未受信時のフォールバック） |
| `torque_radius_recover_threshold` | `0.278159` | `hold` モードで拒否状態から復帰するトルクのしきい値（同上） |
| `max_step` | `0.015` | 1周期あたりの最大変化量 |
| `startup_pose` | `[0, pi/2, 0, pi/2, 0, pi/2]` | 立ち上げ時の通常姿勢 |

`shape_feasibility_node`（teleoperation.launch、`ns=dragon`）:

| パラメータ | 既定 | 説明 |
| --- | --- | --- |
| `robot_model_plugin_name` | `dragon/full_vectoring_robot_model` | 予測に使う DRAGON モデルプラグイン |
| `gimbal_feedback_topic` | `gimbals_ctrl` | `controller` modeで使う最新gimbal指令topic（`ns=dragon` 相対） |
| `baselink_rpy_topic` | `final_target_baselink_rpy` | `controller` / `allocation` modeで使うbaselink目標RPY topic（`ns=dragon` 相対） |
| `baselink_odom_topic` | `uav/baselink/odom` | 任意で使うbaselink実測姿勢topic（`ns=dragon` 相対） |
| `use_gimbal_feedback` | `true` | `controller` modeでgimbal feedbackを使う |
| `use_baselink_rpy_feedback` | `true` | `controller` / `allocation` modeでbaselink RPY feedbackを使う |
| `use_baselink_odom_feedback` | `true` | `controller` / `allocation` modeでbaselink odom姿勢を使う |
| `feedback_timeout` | `0.25` | gimbal/姿勢feedbackの有効期限 [s] |
| `allocation_refine_max_iteration` | `5` | `allocation` modeの静的allocation反復回数 |
| `allocation_refine_threshold` | `0.0001` | `allocation` modeのロータ位置収束判定 |
| `allocation_target_acc` | 未指定 | 6要素を指定するとホバー重力補償の代わりにallocation目標加速度として使う |

`volume_radius_monitor.py`（teleoperation.launch、ライブ監視）:

| パラメータ | 既定 | 説明 |
| --- | --- | --- |
| `enable_shape_safety` | `true` | ライブスケール算出の有効化 |
| `force_inradius_min` / `force_inradius_hard_min` | `0.249220` / `0.108990` | 力のしきい値（帯、4096グリッド分布由来） |
| `torque_inradius_min` / `torque_inradius_hard_min` | `0.278159` / `0.015400` | トルクのしきい値（帯、4096グリッド分布由来） |
| `inradius_timeout` | `0.5` | 内接半径の有効期限 [s] |

モニタリング:

```bash
rostopic echo /dracomancer/dragon_shape_safety_scale   # 安全スケール
rostopic echo /dracomancer/candidate/fc_f_min           # gate前candidateの予測力半径
rostopic echo /dracomancer/candidate/fc_t_min           # gate前candidateの予測トルク半径
rostopic echo /dracomancer/target/fc_f_min              # 最終送信targetの予測力半径
rostopic echo /dracomancer/target/fc_t_min              # 最終送信targetの予測トルク半径
rostopic echo /dragon/debug/fc_f_min                    # 力の内接半径（DRAGON 由来）
rostopic echo /dragon/debug/fc_t_min                    # トルクの内接半径（DRAGON 由来）
```

ログには `state=safe|warning|danger|missing_inradius|disabled`、`force_volume_radius`、`torque_volume_radius`、`safety_scale` を出力します（ログ内の半径値は参照用で、トピックとしては再 publish しません）。

## 力覚提示

`haptics.launch` の `control_haptic_feedback.py` は、精密動作モードで安全スケーリングにより抑制された形状差を用いて、Dracomancer 側の7関節へ返す提示トルク相当量を計算します。ただし、Mk-I で使用している DYNAMIXEL XL430-W250-T は電流制御に対応しておらず、個々のサーボへ所望トルクを入力できません。そのため、Mk-I での既定の力覚提示は、連続的なトルク指令ではなく、しきい値を超えた関節のサーボトルクを ON/OFF する二値提示として扱います。互換サーボや将来機体で使うため、既存の電流指令出力は任意機能として残します。

```text
e_q = q_des - q_tar
tau_q = K_q e_q
tau_device = B^T tau_q - D_h theta_dot
```

ここで `B` は既定の腕関節から DRAGON 6関節への差分マッピングです。実装では、論文補助資料のレンチ分配問題を直接解くのではなく、まず `B^T K_q e_q` による形状空間からデバイス関節空間への直接写像として扱います。

出力:

| トピック | 型 | 説明 |
| --- | --- | --- |
| `/dracomancer/haptic_torque` | `sensor_msgs/JointState` | `effort` に7関節の提示トルク相当量を格納（解析・可視化用） |
| `/servo/torque_enable` | `spinal/ServoTorqueCmd` | 既定出力。提示トルク相当量の絶対値がしきい値以上のサーボをトルク ON |
| `/servo/target_current` | `spinal/ServoControlCmd` | 互換サーボ向けの任意出力。明示的に有効化した場合のみ、提示トルク相当量を電流指令へ変換 |

実機サーボへのトルク ON/OFF 出力は既定で有効です。無効化する場合は `enable_haptic_torque_onoff=false`、ON 判定しきい値を変える場合は `haptic_torque_on_threshold` を調整してください。既存の電流指令を使う場合は、`enable_haptic_current_command=true` と `haptic_current_per_nm` を互換サーボで較正した値に設定します。

```bash
roslaunch dracomancer haptics.launch
```

実機でトルク ON/OFF まで出す例（既定）:

```bash
roslaunch dracomancer haptics.launch \
  haptic_torque_on_threshold:=0.02
```

## 起動方法

DRAGON シミュレーションを動かすPC側:

```bash
roslaunch dragon bringup.launch sim:=true headless:=false
```

Dracomancer / Khadas 側（FC未接続）:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false connect_fc:=false
```

Dracomancer / Khadas 側（FC接続）:

```bash
roslaunch dracomancer bringup.launch rm:=true sim:=false fc_serial_port:=/dev/flight_controller
```

Dracomancer / Khadas 側（FC接続、RVizは親機PCで表示）:

```bash
roslaunch dracomancer bringup.launch \
  rm:=true \
  sim:=true \
  headless:=true \
  fc_serial_port:=/dev/flight_controller
```

親機PC側:

```bash
roslaunch dracomancer rviz.launch
```

RViz には `/dracomancer/servo/states` のサーボtickを `center_tick`（既定2048）基準で換算した各サーボ角度が表示されます。換算式は `(tick - center_tick) * 2π / ticks_per_rev`（`ticks_per_rev` 既定4096）で、表示は deg / rad です。同じMarker内に、`/dracomancer/joint_map/switch_ratio`（`control_joint_angle.py` が publish）から joint1（手首）/joint2（肘）のロール配分比 `rho_i` と pitch/yaw 重み `c_i_pitch`/`c_i_yaw`、配分入力 `r_i` も併せて表示されます（`control_joint_angle.py` が起動していない場合は「waiting for ...」表示）。`servo_labels.py` はいずれかのトピックを受信するたびに即時Markerを更新し、既定では最大30Hzに制限します。受信が `reconnect_timeout`（既定3秒）以上途絶えた場合は購読を自動で張り直します（rospyはWi-Fi瞬断などで切れた購読TCP接続を自動再接続しないため）。更新レート・無通信判定はsim時間（`/clock`）に影響されない壁時計基準です。不要な場合は `show_servo_labels:=false` を指定します。

ROS master は親機PCに固定します。子機PCでは `ROS_MASTER_URI` を親機PCに向け、親機PCでは `ROS_MASTER_URI` を自身に向けた状態で `rviz.launch` のみを起動してください。両PCで `ROS_IP` または `ROS_HOSTNAME` は、それぞれ相手PCから到達可能なIP/ホスト名に設定します。

実機なしで Dracomancer 関節状態を試す:

```bash
roslaunch dracomancer bringup.launch rm:=false sim:=true headless:=false
```

通常のテレオペレーション（別端末で順に起動）:

```bash
roslaunch dracomancer bringup.launch
```

```bash
roslaunch dracomancer teleoperation.launch
```

bringup とテレオペレーションを同じ launch から起動する場合:

```bash
roslaunch dracomancer bringup.launch teleop_mode:=true
```

`teleoperation.launch` を単体で起動した場合、起動直後は `teleop_mode=false` です。ホバリング後、遠隔操作（移動＋腕形状）へ切り替える例:

```bash
rosrun dracomancer set_teleop_mode.py true
```

シミュレーションで形状安全を緩める例:

```bash
roslaunch dracomancer teleoperation.launch \
  enable_feasibility_gate:=false \
  enable_shape_safety:=false \
  publish_joints_only_when_hovering:=true \
  publish_joints_before_device_ready:=false \
  max_step:=0.03
```

IMUを使わない従来方向入力:

```bash
roslaunch dracomancer teleoperation.launch direction_mode:=none
```

ベースリンク移動・前傾反映:

```bash
roslaunch dracomancer teleoperation.launch nav_target:=baselink direction_mode:=yaw_pitch
```

### `bringup.launch` の主な引数

| グループ | 引数 | 既定 | 用途 |
| --- | --- | --- | --- |
| モード | `rm` | `True` | 実機サーボ / Spinal FC を使う |
| モード | `sim` | `False` | RViz表示の既定値に使う |
| モード | `teleop_mode` | `False` | trueなら `teleoperation.launch` も起動し、初期モードを `teleoperation` にする |
| 表示 | `headless` | `True` | モデル表示を隠す |
| 表示 | `launch_rviz` | `$(arg sim)` | RVizを起動する |
| 表示 | `rviz.launch` | - | 親機PCなどGUI環境でRVizだけを起動する |
| Web | `web` | `False` | Web Console を起動する |
| Web | `web_console_port` | `8080` | Web Console のポート |
| Web | `web_console_rosbridge_port` | `9090` | rosbridge のポート |
| FC | `connect_fc` | `$(arg rm)` | Spinal bridge を起動する |
| FC | `fc_serial_port` | `/dev/ttyUSB0` | FC のシリアルデバイス |
| FC | `fc_serial_baud` | `921600` | FC のボーレート |
| FC | `run_servo_rough_calib` | `False` | `servo_rough_calib.py` を起動する |
| サーボ | `servo_topic` | `/dracomancer/servo/states` | `convert_servo_to_joint_states.py` の入力 |
| サーボ | `joint_states_topic` | `/dracomancer/joint_states` | Dracomancer 関節状態の出力 |

`rm=true` では `servo_bridge_node` と `convert_servo_to_joint_states.py` を起動します。`rm=false` では `publish_fake_joint_states.py` を起動し、`sim=true` かつ `web=false` のとき `joint_state_publisher_gui` も起動します。

### `teleoperation.launch` の主な引数

| 引数 | 既定 | 用途 |
| --- | --- | --- |
| `device_ns` | `dracomancer` | Dracomancer 側の名前空間 |
| `robot_ns` | `dragon` | 操縦対象ロボットの名前空間 |
| `enable_joystick_serial` | `false` | rosserial の joystick serial node を起動する |
| `js_raw_topic` | `/joystick/raw` | 操縦桿生値 |
| `js_calibrated_topic` | `/dracomancer/joystick/calibrated` | 較正済み操縦桿 |
| `enable_position_control` | `false` | `/dragon/uav/nav`（操縦桿による移動）を送る。既定 OFF（落下防止のため。下記「既知の課題」参照） |
| `enable_attitude_control` | `false` | `/dragon/final_target_baselink_rpy` を送る |
| `enable_joint_angle_control` | `true` | `/dragon/joints_ctrl` を送る |
| `teleop_mode` | `false` | `false` / `true` |
| `mode_topic` | `/dracomancer/teleop_mode` | 実行中のモード切替トピック（`std_msgs/Bool`） |
| `nav_target` | `cog` | 移動対象 `cog` / `baselink` |
| `direction_mode` | `yaw` | `none` / `yaw` / `yaw_pitch` / `full` |
| `imu_topic` | `/dracomancer/imu` | 操作者IMU |
| `imu_mount_roll/pitch/yaw` | `0 / -1.57079632679 / 0` | IMU取付け補正 |
| `recapture_neutral_on_hover` | `true` | ホバ開始時に中立向きを取り直す |
| `enable_shape_safety` | `true` | ライブshape_safetyスケールとログの有効化。予測ゲートを止める場合は `enable_feasibility_gate:=false` も指定する |
| `publish_joints_only_when_hovering` | `true` | trueならホバリング以降のみ形状指令を送る |
| `publish_joints_before_device_ready` | `false` | falseなら関節状態受信前は送らない |
| `axis_x/y/z` | `0 / 1 / 2` | ジョイスティック軸番号 |
| `xy_vel` | `0.3` | XY速度スケール |
| `z_vel` | `0.2` | Z速度スケール |
| `max_step` | `0.015` | 関節指令のレート制限 |
| `enable_wrist_roll_switching` | `true` | 飽和ロール総和に応じた手首屈曲の `joint1_pitch` / `joint1_yaw` 配分 |
| `wrist_roll_offsets` | `[0.0, 0.0]` | `wrist_roll_joints` と同順の基準角 [rad] |
| `enable_elbow_roll_switching` | `true` | 飽和上腕ロール角に応じた肘屈曲の `joint2_pitch` / `joint2_yaw` 配分 |
| `elbow_roll_offset` | `0.0` | 上腕ロールの基準角 [rad] |
| `enable_link4_anchor` | `true` | link4固定補償を有効化する |
| `link4_anchor_mode` | `position_only` | link4固定の補償方式（`position_only` / `position_yaw` / `full`）。既定はCOG位置のみで補償し、yawは固定しない |
| `enable_link4_anchor_body_safety` | `true` | link4固定ON時にbody補償後の高度・水平距離、full時は姿勢も検査 |
| `enable_link4_anchor_tracking_safety` | `true` | link4固定ON時にCOG/yaw/roll/pitch追従遅れが大きければ形状更新を保持 |
| `enable_link4_anchor_joint_tracking_safety` | `true` | link4固定ON時にDRAGON実関節の追従遅れが大きければ形状更新を保持 |

## 形態目標到達タスク実験（shape_task）

目標形態シャドウへの到達時間・誤差・安全余裕などを記録する評価実験ノード群。
起動は `roslaunch dracomancer shape_task.launch`。キーボード比較条件・dry-run・
ログ形式・評価指標の詳細は [scripts/shape_task/README.md](scripts/shape_task/README.md) を参照。

## デバッグ

各指令の確認:

```bash
rostopic echo /dragon/uav/nav
rostopic echo /dragon/joints_ctrl
rostopic echo /dracomancer/dragon_shape_safety_scale
```

DRAGON がすぐ落下する場合の切り分け:

```bash
roslaunch dracomancer teleoperation.launch enable_joint_angle_control:=false
```

移動指令（操縦桿）は既定で OFF（`enable_position_control:=false`）です。明示的に ON にすると落下する場合は、下記「既知の課題」の `control_position` の FlightNav モード問題に該当します。形状確認だけなら ON にする必要はありません。

```bash
# 移動指令も含めて全て止めて確認する場合
roslaunch dracomancer teleoperation.launch \
  enable_joint_angle_control:=false
```

形状指令を切ると落ちない場合は、`/dragon/joints_ctrl`、`max_step`、`enable_feasibility_gate`、`enable_shape_safety`、`missing_inradius_scale`、関節マッピングを確認してください。

## 実装上の未完了項目

研究仕様に対して、現状で未完了または暫定扱いの項目です。

| 項目 | 現状 |
| --- | --- |
| 操縦桿による移動制御 | 既定 OFF（`enable_position_control:=false`）。`control_position.py` が `POS_VEL_MODE` で `target_pos` 未設定（=0）の FlightNav を送るため、ON にすると DRAGON が原点・高度0へ指令され落下する。`VEL_MODE` 化など修正が必要 |
| 予測フィージビリティ・ゲート | 既定 ON（`enable_feasibility_gate:=true`）。Force/Torque Volume radiusと、同一静的推力配分によるホバーforce・ゼロtorqueの結合可行性に基づき危険姿勢を抑制する。予測fcは既定 `shape_feasibility_prediction_mode=allocation` で候補形状の静的ホバーallocation後のgimbal rollを使い、DRAGON controller側debug fcへ近づける。モデル/設定によって予測 fc が過小になり全変形を却下する場合は、`enable_feasibility_gate:=false` で一時的に無効化して予測器側を調整する |
| 立ち上げ時の通常形状保持 | `teleop_mode=false` で実装。DRAGON 側の preflight joint control 設定に依存 |
| 力覚提示 | 提示トルク相当量の計算とトルク ON/OFF 出力のみ対応。XL430-W250-T では所望電流・所望トルク入力ができないため、Mk-Iでの連続的な反力・反トルク提示は Mk-II 以降の展望。既存の電流指令経路は互換サーボ向け任意機能として保持 |
| Force/Torque Volume に基づく厳密な姿勢制限 | DRAGON の内接半径と結合ホバーwrench可行性を使った形状ゲートを実装。link4アンカーON時は別途COG高度・baselink roll/pitchのbody補償ゲートも実装 |
| C++ controller/model/optimizer | 空のプレースホルダ |
| ジョイスティックZ軸 | `control_position.py` は対応。launch の既定較正は2軸のため、3軸目がなければ0 |
| 位置リミット | `/dragon/mocap/pose` を使い、境界外へ向かう速度成分を0にする形で実装 |

## ビルド

```bash
catkin build aerial_robot_model dracomancer
source devel/setup.bash
```
