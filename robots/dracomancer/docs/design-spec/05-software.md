# 5. システム設計（ソフトウェア）

# 5.1 主要システム①：Human Morphology Acquisition

Dracomancerから得られる人体関節角を用いて人体上肢の各リンク姿勢を算出する。

使用する情報は主として

- 上腕方向
- 前腕方向
- 手先方向
- リンク間相対姿勢
- 曲げ方向
- 腕全体の正規化形状

とする。

**世界座標系における腕全体のtranslation / rotationは、DRAGON base commandへ使用しない。**

したがって操作者が椅子上で多少身体を動かしても、それ自体がロボット全体の移動指令にはならない。

---

# 5.2 主要システム②：固定リンク対応写像（比較手法A）

人間の各仮想リンクとDRAGONの各リンクを固定対応させる。

初期対応：

| Human / Dracomancer | DRAGON |
|---|---|
| 手先方向 | Link 1 |
| 前腕 | Link 2 |
| 上腕 | Link 3 |
| 肩～体幹の仮想リンク | Link 4 |

各隣接リンクの相対姿勢をDRAGON joint Pitch / Yawへ変換する。

概念的には、

\[
q_i^{ref}
=
\arg\min_{q_i}
d_R(R_{robot,i}(q_i),R_{human,i})
\]

とする。

### 特徴

- 直感的な身体対応が明確
- 実装が軽い
- Robot-specificな対応関係を必要とする
- リンク数・関節構造が変わると再設計が必要

本方式は**baseline / comparison method**として残す。

---

# 5.3 主要システム③：全体形状写像（比較手法B・提案）

人体を一度、

> **正規化された3次元link chain / centerline**

として表現し、個々の人体関節とロボット関節の1対1対応を必須としない。

Robot shape と Human shape の差を

\[
E_{shape}(q_R)
\]

として定義し、

\[
q_R^{ref}
=
\arg\min E_{shape}(q_R)
\]

から人間の希望する robot morphology を求める。

### 初期実装

複雑な spline optimizationから始めず、

1. Human chainを同一総長に正規化
2. 各link endpoint / link directionを取得
3. Robot chainとの差を二乗誤差化
4. DRAGON 6 joint DoFを数値最適化

とする。

### Aとの比較目的

| 比較観点 | 固定対応A | 全体形状B |
|---|---|---|
| Human–Robot対応の明瞭性 | 高 | 中 |
| Global shape similarity | 中 | 高を期待 |
| 異なるlink長への対応 | 中 | 高 |
| 異なるlink数への一般化 | 低 | 高 |
| 実装量 | 小 | 中 |
| Flight feasibilityとの統合 | 可能 | より自然 |

**本研究ではAとBを同一入力・同一DRAGON model上で比較する。**

---

# 5.4 主要システム④：Flight-Feasibility Filter

人間形状を直接DRAGONへ送信せず、

\[
q_{human}^{ref}
\rightarrow
(q_R^*, \alpha^*)
\]

という安全補正を行う。

ここで \(\alpha\) はDRAGON thrust-vectoring angleである。

## 設計原則

本研究では「最も飛行性能が高い形」を自動生成することを目的としない。

優先順位は、

1. Hardware / actuator safety
2. Flight feasibility
3. Collision avoidance
4. Human morphology similarity
5. Joint velocity synchronization

とする。

つまり、

> **飛行可能な範囲ではHuman shapeを最大限維持し、必要な場合のみ修正する。**

---

## 5.4.1 先行研究の再利用

Anzaiらは、

- joint limits
- thrust-vectoring limits
- thrust input limits
- joint torque limits
- guaranteed minimum control force \(f_{min}\)
- guaranteed minimum control torque \(\tau_{min}\)

を考慮した transformation planning を既に示している [2]。

本研究では、このモデル・制約計算・solver構成を可能な限り再利用する。

### 本研究との差分

Anzaiら：
> 形態・thrust vectoringを最適化し、flight robustnessを高める。

本研究：
> **Human-commanded morphologyを主目標とし、flight feasibilityを満たさない場合だけ最小限補正する。**

したがって、\(f_{min},\tau_{min}\) は主として「最大化対象」ではなく、**安全margin / constraintとして利用する。**

なお、ここでいう \(\tau_{min}\) は飛行制御可能性を表すcontrol torque marginであり、Valve操作等の**作業トルクそのものとは区別する。**

---

## 5.4.2 Joint velocity

DRAGONの初期研究では、形態変化中にmultilinkを近似的に剛体として扱うためjoint motionを低速に制限し、実験では0.17 rad/sが用いられている [1]。

したがって初期実装では、

- Human morphology velocityを取得
- 同一方向・相対速度関係を維持
- robot-side safe joint velocityでrate limit

する。

「同期」とは必ずしも

> Human 1 rad/s → Robot 1 rad/s

の完全一致ではなく、

> **人間の形状変化の方向・タイミング・相対速度を、飛行可能な速度範囲内で維持する**

ことと定義する。

評価：

- Joint velocity RMSE
- Phase delay
- End-to-end latency
- shape trajectory error

---

# 5.5 主要システム⑤：LiDAR-based Local Collision Constraint

現行DRAGONに搭載されたLiDARを利用する。

LiDAR point cloud から、

- robot linkと障害物の最小距離
- rotor clearance
- passage width
- collision / near-collision

を計算する。

最適化では最低クリアランス

\[
d(q_R,\mathcal P) \ge d_{safe}
\]

を制約として使用する。

## 役割

LiDARは

> **「どこへ行くか」を決めるためではなく、「人間が選んだ動作がその場で実行可能か」を判定する**

ために用いる。

### 実装量削減

Chenらは、point cloudからfloating-base multi-link robotのcollision-free / dynamically feasible trajectoryを生成するframeworkを既に示している [4]。

本研究では、

- raw point cloud処理
- robot geometryとの距離評価
- collision constraint

の利用可能な実装を再利用する。

一方、

- global anchor planning
- autonomous path selection
- trajectory planning全体

は本研究では使用しない。

これにより、環境認識・計画研究へ研究範囲が拡散することを防ぐ。

---

# 5.6 主要システム⑥：Base Pose Operation

ロボット全体位置・姿勢はDracomancerではなくJoystickで操作する。

### Main / Composite Mode

```text
Right arm Dracomancer → Morphology
Joystick              → Position / Attitude
```

を同時に使用する。

これを本研究における **Composite Mode** と定義する。

従来仕様にあった

```text
Human global translation → DRAGON position
Human global rotation    → DRAGON attitude
```

は採用しない。

---

## 補助モード

### Long-distance Mode

- Joystick → Position
- Morphology hold

### Shape Mode

- Dracomancer → Morphology
- base pose hold

### Reference-Link Fixed Mode

指定リンク位置・姿勢を保持しながら、内部形状変化に伴うbase movementを自動補償する。

ただしReference-Link Fixedは第2優先とし、Main Composite Mode完成後に実装する。

### Clutch

Clutch中：

- robot morphology commandを保持
- Humanは腕を楽な姿勢へ戻す
- Release時にHuman referenceを再設定

---

# 5.7 主要システム⑦：Haptic Feedback

## 5.7.1 Contact feedback

初期実験では接触位置を指定EEに限定する。

DRAGONの既存 external wrench estimator を最初に検証し、十分な

- RMSE
- bandwidth
- delay
- noise

が得られる場合はそのまま使用する。

不十分な場合のみEEに6-axis F/T sensorを追加する。

これにより、センサ新規開発を避ける。

---

## 5.7.2 Mapping

Robot EE wrenchを

\[
w_R
\]

とし、Robot joint torqueを \(J_R^T w_R\) とする。

Human–Robot morphology mapping の局所Jacobianを \(M\) とすれば、

\[
\tau_H = M^T J_R^T w_R
\]

を基本とする。

固定対応写像Aと全体形状写像Bでは \(M\) が異なるため、同じwrench feedback frameworkを利用しつつmappingのみ交換可能な設計とする。

---

## 5.7.3 Safety feedback

接触とは別に、

- joint limit
- thrust saturation
- flight feasibility margin
- obstacle proximity
- self collision

への接近を提示する。

ただし修士研究のコアはmapping + feasibilityであり、**Safety hapticの高度なnull-space設計は必須としない。**

まずは、

- spring-like torque
- vibration
- directional resistance

のいずれか簡単な方式から検証する。

Aerial teleoperationではhaptic feedbackによってcontactやremote stateを提示する研究が存在し [5][6]、一般teleoperationでも触覚feedbackの安全性・情報提示能力が整理されている [7]。したがって、haptic interface全体を新規理論化するのではなく、多関節形態写像への適用部分のみを研究対象とする。

---

# 5.8 Software Architecture

新規実装は以下に分割する。

```text
dracomancer_hardware
    └─ joint state / actuator I/O

human_morphology
    └─ virtual human chain

morphology_mapping
    ├─ fixed_link_mapping
    └─ global_shape_mapping

morphology_feasibility
    ├─ flight_feasibility
    ├─ lidar_collision_constraint
    └─ joint_rate_limiter

teleop_mux
    ├─ joystick base command
    ├─ web client command
    ├─ morphology command
    └─ clutch / mode

web_client
    ├─ state visualization
    ├─ mode / clutch operation
    ├─ low-speed base operation
    ├─ experiment operation
    └─ log playback / shared view

log_platform
    ├─ ROS bag ingestion
    ├─ topic extraction / time synchronization
    ├─ web playback data storage
    └─ share URL / access control

haptic_controller
    ├─ contact feedback
    └─ safety feedback

dracomancer_bringup
```

## Middleware方針

ROS 2 native化そのものは本研究の学術的寄与ではないため、

> **既存DRAGON softwareが安定して動作する環境を優先する。**

必要に応じ、

- ROS 1 existing DRAGON stack
- ROS bridge
- 新規Dracomancer側のみROS 2

などの段階的構成を許容する。

DRAGON全体のROS 2 portを研究完了条件とはしない。

---

# 5.9 補助システム：Web Client

Webクライアントは研究用アルゴリズムから分離し、ブラウザUIとROS側gatewayで構成する。既存の`aerial_robot_web`を再利用できる場合は優先して利用し、Web技術自体の新規開発を研究課題にしない。

主な表示項目は次とする。

- DRAGONの飛行状態、base pose、内部関節角
- Human desired shapeと補正後のfeasible morphology
- \(f_{min}\)、\(\tau_{min}\)、推力飽和、最小障害物距離
- Dracomancer、Joystick、LiDAR、通信の接続状態
- 現在の操作モード、Clutch、安全制約による補正・拒否状態
- アップロード済み実験ログの3D動作再生と時系列グラフ

主な操作項目は次とする。

- 操作モード選択とClutch
- 低速なbase position / attitude command
- 実験の開始・停止とログ記録
- 緊急停止要求

## 5.9.1 実験ログの可視化・共有基盤

初期対応形式はROS 1のbagファイルとする。将来的にrosbag2やCSV等を追加できるよう、入力形式に依存する変換処理とブラウザ向け再生データを分離する。

```text
ROS bag / experiment log
          │
          ↓
  Upload / Import API
          │
          ↓
Topic validation / extraction
          │
          ↓
Time-synchronized playback data
          │
     ┌────┴────┐
     ↓         ↓
3D motion   Time-series plots
     └────┬────┘
          ↓
 Read-only shared URL
```

最低限、次のデータを再生対象とする。

- DRAGONのbase pose、内部関節角、目標形態
- Dracomancerの人体関節角とHuman desired shape
- 補正後のfeasible morphology
- \(f_{min}\)、\(\tau_{min}\)、推力飽和、障害物距離
- flight state、制約違反、gate rejection、failsafe等のイベント

ブラウザ表示は、URDF等から構成した3Dモデル、時系列グラフ、イベント一覧を同じ再生時刻へ同期する。再生、一時停止、任意時刻へのシーク、再生速度変更、表示topic選択を提供する。

ログ変換後のデータにはschema versionと元ログのhashを記録し、同じログから同じ表示を再生成できるようにする。共有URLは閲覧専用を既定とし、公開範囲、失効日時、失効操作を設定できるようにする。非公開topic、操作者の個人情報、カメラ映像、位置情報を共有対象へ含める場合は、アップロード前または公開前に明示的な確認を要求する。

大容量bagをブラウザへ直接読み込ませることを前提とせず、サーバ側またはローカル変換ツールで必要topicを抽出・軽量化する。保存先、認証方式、URL発行方式、最大ファイルサイズ、ログ保持期間は、利用するホスティング環境を決定した後に確定する。

## 5.9.2 操作経路の安全要件

安全上、ブラウザからDRAGONへ制御topicを直接publishしない。ROS側gatewayは許可するcommandを限定し、入力値検証、rate limit、command timeout、操作者の明示的なenable、緊急停止時の安全側遷移を実装する。ネットワーク越しに公開する場合の認証・暗号化方式は、利用環境を確定してから定める。

Webクライアントは補助機能であり、完成しなくてもコア研究の成立条件には含めない。

---

## 参考文献

[1] M. Zhao, T. Anzai, F. Shi, X. Chen, K. Okada, and M. Inaba,
“Design, Modeling, and Control of an Aerial Robot DRAGON: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation,”
*IEEE Robotics and Automation Letters*, vol. 3, no. 2, pp. 1176–1183, 2018.
DOI: 10.1109/LRA.2018.2793344.

[2] T. Anzai, M. Zhao, M. Murooka, F. Shi, K. Okada, and M. Inaba,
“Design, Modeling and Control of Fully Actuated 2D Transformable Aerial Robot with 1 DoF Thrust Vectorable Link Module,”
*2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2820–2826, 2019.

[4] Y. Chen, J. Li, H. Liu, Z. Luo, K. Kaneko, and M. Zhao,
“Hierarchical Trajectory Planning of Floating-Base Multi-Link Robot for Maneuvering in Confined Environments,”
*IEEE Transactions on Automation Science and Engineering*, 2026.
DOI: 10.1109/TASE.2026.3669051.

[5] M. Allenspach, N. Lawrance, M. Tognon, and R. Siegwart,
“Towards 6DoF Bilateral Teleoperation of an Omnidirectional Aerial Vehicle for Aerial Physical Interaction,”
*IEEE International Conference on Robotics and Automation (ICRA)*, pp. 9302–9308, 2022.

[6] M. Macchini, T. Havy, A. Weber, F. Schiano, and D. Floreano,
“Hand-Worn Haptic Interface for Drone Teleoperation,”
*IEEE International Conference on Robotics and Automation (ICRA)*, pp. 10212–10218, 2020.

[7] C. Pacchierotti and D. Prattichizzo,
“Cutaneous/Tactile Haptic Feedback in Robotic Teleoperation: Motivation, Survey, and Perspectives,”
*IEEE Transactions on Robotics*, vol. 40, pp. 978–998, 2024.
DOI: 10.1109/TRO.2023.3344027.
