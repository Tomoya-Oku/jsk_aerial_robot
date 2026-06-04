# 03. 制御

DRAGON には 2 つの制御方式があり、`full_vectoring_mode`（既定 true）で選択します。

```mermaid
flowchart TD
    SEL{"full_vectoring_mode"}
    SEL -->|true（推奨）| FV["DragonFullVectoringController"]
    SEL -->|false| LQI["DragonLQIGimbalController"]
```

## フルベクタリング制御（既定）

`DragonFullVectoringController`（`PoseLinearController` を継承）は、
目標 6 自由度加速度／レンチを、**各ロータの推力**と**ジンバル角度**に配分します。

```mermaid
flowchart TB
    POSE["位置/姿勢 誤差"] --> ACC["目標加速度 (6DoF)<br/>target_acc_cog"]
    EXT["外力推定<br/>externalWrenchEstimate"] --> WRENCH["目標レンチ<br/>target_wrench_cog"]
    ACC --> WRENCH
    WRENCH --> ALLOC["推力 / ジンバル配分<br/>(アロケーション)"]
    MODEL["FullVectoringRobotModel"] --> ALLOC
    ALLOC --> COMP["ロータ干渉補償<br/>rotorInterfereCompensation"]
    COMP --> CMD["sendCmd"]
    CMD --> FCMD["flight_cmd（推力）→ spinal"]
    CMD --> GIM["gimbal_control（ジンバル角）"]
```

### アロケーション手法（3 種）

目標レンチからアクチュエータ指令を解く方法が 3 つ用意されています。

```mermaid
flowchart LR
    W["目標レンチ"] --> M1["静的反復<br/>staticIterativeAllocation"]
    W --> M2["厳密非線形<br/>strictNonlinearAllocation"]
    W --> M3["勾配降下<br/>gradientDescentAllocation"]
    M1 --> SR["SR-inverse による解<br/>srInverseAllocation"]
    M2 --> OUT["推力・ジンバル角"]
    M3 --> OUT
    SR --> OUT
```

| 手法 | フラグ | 特徴 |
| --- | --- | --- |
| 静的反復 | `enable_static_allocation_method_` | 反復で擬似逆解を改善 |
| 厳密非線形 | `enable_nonlinear_allocation_method_` | NLopt による非線形最適化 |
| 勾配降下 | `enable_gradient_allocation_method_` | 勾配法 |

### 外力推定とロータ干渉補償

```mermaid
flowchart LR
    MOM["運動量オブザーバ<br/>momentum_observer"] --> EW["外力レンチ推定<br/>est_external_wrench"]
    EW --> PUB["estimate_external_wrench（公開）"]
    EW --> COMP["レンチ補償項へ反映"]
    OVL["ロータ/リンク重なり判定<br/>overlap_*_thresh"] --> RI["干渉推力補償<br/>rotor_interfere_compensate"]
```

- `addExternalWrench` / `clearExternalWrench` サービスで外力を手動付加（試験用）。
- `extra_vectoring_force` で追加のベクタリング力を注入可能。

### 公開デバッグトピック（抜粋）

| トピック | 説明 |
| --- | --- |
| `target_vectoring_force` | 配分後のベクタリング力 |
| `estimate_external_wrench` | 推定外力レンチ |
| `rotor_interfere_wrench` | ロータ干渉補償レンチ |
| `debug/fc_f_min` / `debug/fc_t_min` | 力 / トルクの実現可能内接半径 |

## LQI ジンバル制御（旧方式）

`DragonLQIGimbalController` は、**roll/pitch/z を LQI**、**x/y/yaw をジンバル**で扱う方式です。
hydrus 系の LQI コントローラを DRAGON のジンバルへ拡張したものです。

```mermaid
flowchart LR
    ERR["状態誤差"] --> LQIB["LQI: roll / pitch / z<br/>(推力)"]
    ERR --> GIMB["gimbal: x / y / yaw"]
    LQIB --> CMD2["推力指令 → spinal"]
    GIMB --> CMD3["ジンバル角指令"]
```

## 制御設定ファイル

| ファイル | 用途 |
| --- | --- |
| `control/FullVectoringControlConfig.yaml` | フルベクタリング制御パラメータ |
| `control/LQIGimbalControlConfig.yaml` | LQI ジンバル制御パラメータ |

> シミュレーションでは `controller/rotor_interfere_compensate=false`（ロータ干渉補償オフ）が
> 自動で設定されます。
