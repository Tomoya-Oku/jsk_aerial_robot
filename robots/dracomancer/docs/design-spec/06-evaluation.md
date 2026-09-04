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

---

## 参考文献

この章で直接参照する文献はありません。
