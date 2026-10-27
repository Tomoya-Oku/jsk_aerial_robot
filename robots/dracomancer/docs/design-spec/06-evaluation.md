# 6. 検証（実験）環境・実施タスク・評価項目

## 6.1 実験環境

### Robot

- DRAGON
- 4 links
- 3 joints × Pitch/Yaw = 6 internal DoF
- onboard LiDAR
- onboard camera
- existing flight controller

### Operator

- 座位
- Right-arm Dracomancer Mk-II
- Joystick
- Robot image / configuration visualization

### Ground Truth

必要に応じ、

- OptiTrack
- 6-axis F/T sensor

を使用する。

OptiTrackは操作者へのナビゲーション情報としては使用せず、評価用Ground Truthとする。

---

## 6.2 Experiment 1：Mapping Comparison

### 目的

固定リンク対応と全体形状写像の違いを明確にする。

### 条件

- Method A：Fixed-link correspondence
- Method B：Global-shape matching

### タスク

複数の代表形状を連続的に生成する。

例：

- Straight
- L-shape
- S-shape
- 3D bend
- Narrow-passage target shape

### 評価

- Normalized shape RMSE
- DRAGON joint-space coverage
- Target shape completion time
- Joint velocity RMSE
- Phase delay
- End-to-end latency
- 操作者の主観的対応の分かりやすさ
- NASA-TLX

### 重要点

この実験ではflight feasibility filterを同条件に固定し、**mapping methodそのものの差**を評価する。

---

## 6.3 Experiment 2：Feasibility-aware Morphology

### 目的

人間形状の最大模倣と飛行可能性保証が両立できるか検証する。

### 比較

- Human mappingのみ
- Human mapping + Flight-feasibility filter

### 実施順

1. Simulation
2. Tether / safety setup
3. Real flight

### 評価

- Human–Robot shape error
- \(f_{min}\)
- \(\tau_{min}\)
- thrust saturation rate
- joint torque constraint violation
- singular / infeasible state count
- Position / attitude tracking error

なおこの比較は主としてSimulationで実施し、実機で危険な「filter OFF」条件を無理に行わない。

---

## 6.4 Experiment 3：LiDAR Narrow Passage

### タスク

操作者が映像を見ながら進行方向を決め、腕を細長く変形させて開口部を通過する。

```text
START
  │
  ▼
DRAGON
  │
  ▼
┌────────┐
│ Narrow │
│ Passage│
└────────┘
  │
  ▼
 GOAL
```

### 比較

- Mapping only
- Mapping + LiDAR local collision constraint

必要に応じて最終的には最も性能の良いmapping methodのみ使用し、比較条件数を抑える。

### 評価

- Success rate
- Completion time
- Collision count
- Minimum clearance
- Human–Robot shape error
- optimizer correction amount
- flight feasibility margin
- NASA-TLX

---

## 6.5 Experiment 4：統合タスク

### Narrow Passage + High-Torque Manipulation

```text
START
  ↓
狭隘部へ接近
  ↓
腕を伸ばす / 曲げる
  ↓
LiDAR-assisted morphology correction
  ↓
狭隘部通過
  ↓
再変形
  ↓
EEをValveへ接触
  ↓
操作者が高トルク形態を選択
  ↓
Valve rotation
```

### 高トルク形態

本研究ではoptimizerが自動的に「最大トルク形態」を選択しない。

> **高トルクになりやすい形態を人間自身が選択する。**

これにより、

- 環境・タスクの判断：Human
- 実現可能性保証：Robot

という研究コンセプトを維持する。

---

## 6.6 統合タスク評価

### 操作性能

- Success rate
- Completion time
- Collision count
- Mode switch count
- Required shape changes

### Shape / Motion

- Shape RMSE
- Joint angle RMSE
- Joint velocity RMSE
- Phase delay

### Flight

- Position RMSE
- Attitude RMSE
- Minimum \(f_{min}\)
- Minimum \(\tau_{min}\)
- thrust saturation rate

### Narrow-space

- Minimum clearance
- Collision / near-collision count
- LiDAR correction magnitude

### Manipulation

- Peak contact force
- Peak contact torque
- Valve rotation angle
- task success

### Human factors

- NASA-TLX
- 操作の分かりやすさ
- 身体負荷
- 装着性

---

## 6.7 Haptic Experiment（第2優先）

時間に余裕がある場合のみ、

- Haptic OFF
- Haptic ON

を比較する。

評価：

- Peak contact force
- Force overshoot
- contact task success
- reaction time
- NASA-TLX

Haptic experimentをmapping比較と全factorialに組み合わせない。
**Hapticは最終的に選定した1種類のmapping methodで評価する。**

これにより条件数と被験者実験量を抑える。

### 壁押し接触シミュレーション

本環境について、以下を確定条件とする。

- 壁はワールドに固定する剛体とし、DRAGONに押されても移動・転倒しない。
- 作業点はDRAGONのLink 1先端とし、壁から受ける反力・反トルクをDracomancerへ提示する。
- 形態対応は既存の`control_joint_angle.py`を参照する。肩・肘・手首の対応に加え、ロール角によるPitch/Yaw配分を含む。
- 実機デバイスと通信経路は[4.6 Actuator / Sensor](04-hardware.md#46-actuator--sensor)に従う。
- シミュレータは既存のGazebo（ODE）を使用する。
- 壁押しタスク中はLink 4位置固定を解除する。
- 接触力の取得元と提示倍率・上限は[5.7 Haptic Feedback](05-software.md)に従う。
- 固定解除後はJoystickによる機体移動と腕の形態操作を併用する。
- 壁の中心はワールド座標[1.5, 0, 1.5] m、寸法は[厚さ0.1, 幅3, 高さ3] m、法線はX方向、摩擦係数は0.8とする。

配置・寸法・力覚設定は`config/contact.yaml`から変更する。追加ツールは使わず、Link 1の既存先端面を接触させる。

#### 起動と操作

ROS Noetic / Gazebo Classic 11と、ルートREADMEの外部依存を用意する。
`catkin build spinal dracomancer`後、新しいros_libを使用してSpinalとNeuron両方のファームウェアを再ビルド・書込みする。
ファームウェアの書込みは本環境の起動処理には含めない。

```bash
source devel/setup.bash
roslaunch dracomancer contact.launch fc_port:=/dev/ttyUSB0
```

起動直後は力覚OFF、teleop OFFである。`hardware:=false`を付けると実機へ接続・電流出力せず、
Dracomancer関節角は既存の`/dracomancer/joint_cmd`から入力できる。
既存の`haptics.launch`や、別の`bringup.launch`・`teleoperation.launch`を重複起動しない。
既存のJoystick生入力は`/joystick/raw`を使う。保存済みの2軸校正を利用し、XY移動を行う。
Z軸がない入力では高度速度を0にして現在高度を保持し、姿勢は既存飛行制御に任せる。
Joystickの再配信で入力途絶が隠れないよう、生入力と移動ノード双方で100 msを監視する。

別端末から既存のキーボード操作を起動し、`r`でモータ始動、`t`で離陸し、HOVERを確認する。

```bash
rosrun aerial_robot_base keyboard_command.py _robot_ns:=/dragon
rostopic pub -1 /dracomancer/teleop_mode std_msgs/Bool 'data: true'
# 新鮮な関節・接触・時刻データがそろってから有効化する。
rosservice call /contact_haptics/enable 'data: true'
```

腕で先端面を壁側へ向け、Joystickで接近して押し付ける。
Link 4固定は無効であり、形態操作ノードはアンカー用の位置指令を送らない。
移動ノードは`VEL_MODE`を使用し、未指定の目標位置0を送らない。

```bash
# 停止・再開とも操作者の明示操作。停止後にデータが復旧しても自動再開しない。
rosservice call /contact_haptics/enable 'data: false'
rosservice call /contact_haptics/enable 'data: true'
```

記録対象は`/dragon/contact/wrench`、`/dracomancer/contact/torque`（±0.2 N·m制限後・電流量子化前の要求トルク）、
`/dracomancer/contact/enabled`、`/dracomancer/mapping_state`、両機の`joint_states`、`/clock`とする。
`/dracomancer/servo/target_current`は2.69 mA単位の指令値を`spinal/ServoControlCmd.angles`に格納する既存規約を使う。
サーボ順は既存`Servo.yaml`のSpinal index 0～6であり、物理DYNAMIXEL IDではない。

#### 再現可能な検証

```bash
python3 -B robots/dracomancer/test/test_contact.py
python3 -B robots/dracomancer/test/test_haptics.py
g++ -std=c++17 aerial_robot_nerve/test/current_guard.cpp -o /tmp/current_guard
/tmp/current_guard
# 他のGazebo/ROSセッションを起動していない隔離環境で実行する。
bash robots/dracomancer/test/gazebo.sh
# 上の隔離テストを終了してから統合環境を起動する（実機接続なし）。
roslaunch dracomancer contact.launch hardware:=false headless:=true
# 別端末で実行。模擬関節入力、離陸、形態変更、機体旋回、Joystick入力で接触させ、終了時に着陸する。
python3 -B robots/dracomancer/test/teleop_smoke.py --push
```

Gazeboテストでは、先端面を壁へ1 Nで押す単一剛体を用い、固定関節縮約を含めた反力の座標・符号・重心オフセットのモーメントと物理停止を検証する。
統合試験は`hardware:=false`の場合だけ動作する。先端が壁を向く形態・機体姿勢にし、他リンクが先に壁へ当たらないよう操縦する必要がある。
先端面以外の接触は物理反力を生むが、力覚には含めない。

2026-10-27、隔離したROS Noetic / Gazebo 11環境で、依存を含む16パッケージのCatkinビルド、
数値・入力時刻・ROS停止監視の15テスト、MCU電流ガードのホストテスト、ODEの1 N接触試験が成功した。
DRAGON統合試験ではHOVER、既存形態写像の6×7 Jacobian、Joystick経由の壁接近、先端反力から非ゼロの力覚要求、
7軸の±0.2 N·m制限、物理停止時の無効化と明示的再有効化を確認した。
接触瞬間には大きいODE反力が生じ得るため、要求トルクは上限制限後の値も記録する。

Spinal（H7 v2）とNeuron（F4）の変更6翻訳単位は、生成したros_libとARMコンパイラで構文検査済みである。
再実行には`arm-none-eabi-g++`を用意し、以下を使う。

```bash
python3 -B aerial_robot_nerve/test/syntax.py --ros-lib aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/ros_lib
```

ファームウェアのリンク・書込み、実機のモード読戻し・電流応答・通信断停止、実トルク係数、力覚を含む閉ループ安定性は未確認である。
上記の試験成功は全形態の飛行安定性や実機出力トルクを保証するものではない。

---

## 参考文献

この章で直接参照する文献はありません。
