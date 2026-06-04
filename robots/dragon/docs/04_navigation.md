# 04. ナビゲーション

`DragonNavigator`（`BaseNavigator` を継承）は、飛行状態の遷移管理に加え、
DRAGON 特有の **機体姿勢（baselink rotation）制御**・**ジンバル制御**・
**変形時のサーボトルク管理**を担当します。

## フライトステート遷移

```mermaid
stateDiagram-v2
    [*] --> ARM_OFF
    ARM_OFF --> START: アーム要求
    START --> ARM_ON
    ARM_ON --> TAKEOFF: 離陸
    TAKEOFF --> HOVER: 高度到達
    HOVER --> LAND: 着陸要求
    LAND --> PRE_LAND: 機体を水平化
    PRE_LAND --> LAND: 水平化完了
    LAND --> ARM_OFF: 着地
```

- DRAGON 固有の追加状態 **`PRE_LAND_STATE (0x20)`**: 着陸前に機体を水平へ整える。
- 着陸処理では高度 `height_thresh`（既定 0.1 m）以下でサーボトルクを切る。

## 機体姿勢（baselink rotation）処理

DRAGON は変形に応じて重心や姿勢が変わるため、目標 baselink 姿勢を spinal へ送ります。

```mermaid
flowchart LR
    RPY["final_target_baselink_rpy<br/>(Vector3)"] --> PROC["baselinkRotationProcess"]
    QUAT["final_target_baselink_rot<br/>(Quaternion)"] --> PROC
    PROC -->|変化量 > thresh<br/>間隔ごと| PUB["target_baselink_rpy → spinal"]
```

| パラメータ | 説明 |
| --- | --- |
| `baselink_rot_change_thresh` | 姿勢更新の最小変化量（既定 0.04） |
| `baselink_rot_pub_interval` | 姿勢配信間隔（既定 0.2 s） |

## 入出力トピック

```mermaid
flowchart TB
    subgraph IN["購読"]
        I1["uav/nav（移動指令）"]
        I2["final_target_baselink_rpy"]
        I3["final_target_baselink_rot"]
        I4["joints_ctrl（形状指令）"]
    end
    NAV["DragonNavigator"]
    subgraph OUT["配信"]
        O1["target_baselink_rpy → spinal"]
        O2["joint_control（関節指令）"]
        O3["gimbal 制御"]
    end
    I1 --> NAV
    I2 --> NAV
    I3 --> NAV
    I4 --> NAV
    NAV --> O1
    NAV --> O2
    NAV --> O3
```

## プリフライト関節制御

`enable_preflight_joint_control`（シミュレーションでは既定 true）が有効なとき、
離陸前から関節指令を反映できます。実機では地上での意図しない変形を防ぐため既定 false。

## 変形デモ

ホバリング後（`Hovering!` 表示後）に `transformation_demo.py` で定型姿勢へ変形できます。

```mermaid
flowchart LR
    DEMO["transformation_demo.py"] --> JC["/dragon/joints_ctrl"]
    DEMO --> ATT["/dragon/final_target_baselink_rpy"]
```

| コマンド | 形状 |
| --- | --- |
| `rosrun dragon transformation_demo.py _mode:=0` | シーホース姿勢 |
| `rosrun dragon transformation_demo.py _mode:=1` | スパイラル姿勢 |
| `rosrun dragon transformation_demo.py _mode:=2` | M 字姿勢 |
| `rosrun dragon transformation_demo.py _reset:=1` | 通常姿勢へ復帰 |
| `rosrun dragon transformation_demo.py _reverse_reset:=1` | 反転通常姿勢 |

## 遠隔操作

| 方法 | 参照 |
| --- | --- |
| キーボード | [wiki: keyboard_operation](https://github.com/JSKAerialRobot/aerial_robot/wiki/keyboard_operation) |
| ジョイスティック | [wiki: joystick_operation](https://github.com/JSKAerialRobot/aerial_robot/wiki/joystick_operation) |
| Dracomancer（外骨格） | [dracomancer/docs](../../dracomancer/docs/README.md) |

主なナビゲーション設定（`NavigationConfig.yaml`）:

| パラメータ | 既定 | 説明 |
| --- | --- | --- |
| `takeoff_height` | 1.0 m | 離陸高度 |
| `max_target_vel` | 0.5 | 目標速度上限 |
| `max_target_yaw_rate` | 0.1 | ヨーレート上限 |
| `xy_control_mode` | 0 | 0=位置, 2=速度(world), 3=速度(local), 4=姿勢 |
