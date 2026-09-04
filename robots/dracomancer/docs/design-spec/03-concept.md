# 3. 概要（コンセプト）・システム全体像

## 3.1 基本コンセプト

本システムでは、操作者の上肢を **ロボット全体の位置・姿勢入力には使用しない。**

```text
                     Human Operator
                           │
              ┌────────────┴─────────────┐
              │                          │
        Dracomancer Mk-II            Joystick
              │                          │
       相対的な腕形状                Base motion
              │                  Position / Attitude
              ↓                          │
      Human Morphology                   │
              │                          │
              ↓                          │
      Morphology Mapping                 │
   ┌──────────┴──────────┐               │
   │                     │               │
固定リンク対応       全体形状写像        │
   │                     │               │
   └──────────┬──────────┘               │
              ↓                          │
       Human desired shape q_ref         │
              │                          │
              ↓                          │
   Feasibility / Collision Filter        │
       ↑                 ↑               │
 DRAGON model        LiDAR point cloud   │
              │                          │
              ↓                          │
     Feasible morphology q*              │
              └────────────┬─────────────┘
                           ↓
                         DRAGON
                           │
                 state / wrench / limits
                           ↓
                    Haptic Feedback
                           ↓
                    Dracomancer Mk-II
```

---

## 3.2 操作自由度の分担

| 操作対象 | 入力方法 | 備考 |
|---|---|---|
| DRAGON内部形状 | Dracomancer Mk-II | 主研究対象 |
| 内部形状変化速度 | Dracomancer Mk-II | 安全速度範囲で同期 |
| XY位置 | Joystick | 腕全体移動は利用しない |
| Z位置 | Joystick / Trigger | 実装しやすい方式を採用 |
| Roll / Pitch / Yaw | Joystick等 | 必要な姿勢自由度のみ |
| Clutch | Button | 基準姿勢の再設定 |
| 障害物情報 | DRAGON LiDAR | 操作者の代わりに経路を決めない |
| 接触情報 | wrench estimator / F/T sensor | 力覚提示用 |

---

## 3.3 DRAGONへの適用

対象機体は4リンクDRAGONとし、内部形状を

\[
\mathbf q_R =
[q_{1p},q_{1y},q_{2p},q_{2y},q_{3p},q_{3y}]^T
\]

とする。

各 Pitch / Yaw の可動域は原則

\[
-90^\circ \le q_{ip}, q_{iy} \le 90^\circ
\]

とする [1]。

本研究ではロータ thrust / gimbal control 自体を新規設計せず、DRAGONの既存 flight controller を使用する。

---

## 3.4 人間とロボットの役割分担

### 人間が担当すること

- 進行方向の判断
- どの隙間を通るかの判断
- 狭隘部に適した基本形状の生成
- 高トルク接触作業に適した基本形状の生成
- 接触作業時の操作判断

### ロボット側が担当すること

- 飛行安定化
- 推力配分
- thrust vectoring
- 関節・推力制約の監視
- LiDARによる局所衝突判定
- 人間形状から最小限の形態修正
- 接触・安全状態のフィードバック

したがって、本研究は完全自律化ではなく、

> **Human intention + Robot feasibility assistance**

というShared-control構成とする。

## 3.5 Webブラウザによる補助操作

Dracomancer Mk-IIとJoystickによる主操作に加え、任意の端末のWebブラウザから利用できるWebクライアントを補助UIとして用意する。

Webクライアントが担当する機能は次を基本とする。

- DRAGON、Dracomancer、飛行可能性、障害物距離の状態監視
- 操作モードの選択とClutch操作
- Joystickを使用できない場合の低速なbase pose操作
- 実験開始・停止、記録、緊急停止要求などの運用操作
- ROS bag等の実験ログの読み込みと、ロボット・操作者・安全状態の時刻同期再生
- 再生結果を閲覧専用URLとして発行し、ブラウザだけで共有・確認できる機能

Webクライアントからの指令はJoystickと同じ`teleop_mux`へ入力し、通信断監視、速度制限、flight-feasibility filter、collision constraintを迂回できない構成とする。形態操作の主入力は引き続きDracomancer Mk-IIとし、Webブラウザ上での関節個別操作は調整・保守用途に限定する。

ログ再生では、DRAGONのbase pose・内部関節、Dracomancerの人体関節、形態指令、飛行可能性、安全イベントを同じ時間軸で描画する。3D表示と時系列グラフを連動させ、再生、一時停止、シーク、速度変更、重要時刻へのジャンプを可能にする。

この機能は研究の本筋および主要な比較実験には含めず、システムの運用性、結果説明、共同研究者との共有を支える付加機能として扱う。

---

## 参考文献

[1] M. Zhao, T. Anzai, F. Shi, X. Chen, K. Okada, and M. Inaba,
“Design, Modeling, and Control of an Aerial Robot DRAGON: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation,”
*IEEE Robotics and Automation Letters*, vol. 3, no. 2, pp. 1176–1183, 2018.
DOI: 10.1109/LRA.2018.2793344.
