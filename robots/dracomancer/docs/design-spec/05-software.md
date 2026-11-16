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

本方式は研究上の**baseline / comparison method**とする。旧Mk-I実装は削除し、Mk-II用の実装・校正は別途行う。

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

### ROS 1初期実装（2026-11-16）

`teleoperation.launch mapping_mode:=shape` で、Mk-II CAD URDFの右腕中心線と
DRAGON URDFのLink4尾端→Link1中心線を等弧長・全長正規化して最小二乗で写像する。
仮想体幹点は通常設定では未設定とし、有効化を拒否する。
`shape_demo.launch`と`contact.launch`のシミュレーションでは、指示者が承認したbase_link原点の仮設定を使う。
肩・肘・手首には設定されたCADフレーム、手先には手先リンクのCAD重心を代表点として使う。
これは解剖学的な関節中心の校正ではない。

数値最適化は既定10Hz、指令ループは40Hz。採用済み指令をwarm startとして、
有限差分ヤコビアン、連続性・参照形態の正則化、関節境界への投影、バックトラッキングを用いる。
既存holdゲートで候補を検査し、速度制限後の発行形態も再検査する。
異常入力・計算時間超過・検査失敗時は最後の発行指令を保持する。
離散的な形態検査であり、経路全体・動的安定性の保証や安全集合への最近点射影ではない。

起動、診断、検証結果と残る制約は[実装・検証記録](../shape.md)を参照する。

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

ROS 1実装では既存`max_step`に加え、発行指令を実経過ROS時間に基づき制限する。
既定値0.1rad/sは現在のv1.5 URDFの関節速度属性を採用した値で、実機の安全認証値ではない。
shapeでは設定上限と読込URDFの軸別上限を交差させる。停止・再開・時計巻戻り後に遅延分を加算しない。
実関節の追従性能・動的安全性は別途評価する。

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

## 5.7.1 接触・安全力覚の設計

接触点は指定EEに限定し、既存external wrench estimatorのRMSE・帯域・遅延・ノイズを検証する。
不十分な場合に限りF/Tセンサを追加する。壁押しシミュレーションではGazebo ODEの接触真値を別の評価入力として使う。
接触wrenchとJacobianは同一座標系・同一モーメント基準点で表す。

人体7関節からDRAGON6関節への局所写像JacobianをM、ロボット関節速度から接触点twistへのJacobianをJ_Rとすると、
接触力覚の基本式は次とする。

\[
\tau_H^{contact}=M^TJ_R^Tw_R
\]

形態の生指令と可行指令の差をe_Rとし、安全力覚は次の抵抗・減衰を基本とする。

\[
\tau_H^{safe}=-M^TKe_R-B\dot q_H
\]

接触・安全力覚を合成し、関節別のトルク上限・変化率制限を適用する。
既存研究 [5][6][7] を踏まえ、これらの高度な理論化は研究の必須条件としない。
汎用の計算部品は`scripts/haptics/contact_feedback.py`・`safety_feedback.py`に保持する。

## 5.7.2 実機の出力経路

Mk-IIの実機出力は`scripts/mkii/direct.py`を単一の所有者とする。
`torque_cmd`のNm要求を校正済みの`dracomancer.xm430.CurrentScale`で電流へ変換し、
Spinalの`direct_servo_mode`・`servo/direct_current`を使う。
電流単位2.69 mAとXM430の仕様は [10] を参照する。
実機設定は[4.6](04-hardware.md#46-actuator--sensor)、準備・HOLD・解放・復旧は[実機手順](../hardware.md)に従う。

同一ロボットPC・同一ユーザー・同じ一時ディレクトリとROS_MASTER_URIを使う直結ノードは、
出力前に`dracomancer.output_lock`で電流トピックとモードサービスを排他取得する。
後発ノードはpublish・終了処理の登録前に停止し、動作中の所有者へ0電流・OFFを送らない。
MCU側の電流セッション・期限監視も維持する。複数PC間のファイルロックによる排他は実装しない。

## 5.7.3 現在の対応範囲

Mk-IIの角度表示・Gazeboの角度／トルク操作・直結実機の手動トルク操作を統合する。
形態操作はCAD中心線を用いるshapeモードのみとし、旧Mk-Iの解剖学的関節名による写像・CAN力覚出力を削除する。
主名前空間は`/dracomancer`、モデル・STL・Python部品・起動ファイルは同名の1パッケージへ集約する。

shapeの局所Jacobian Mと接触Jacobian J_Rから直結力覚へ送る経路は未実装である。
このため、shapeから接触力覚を自動出力せず、旧MappingStateや固定対応行列をCAD関節へ流用しない。
`contact.launch`はシミュレーションで接触wrenchを観測する構成に限定し、実機・力覚出力を起動しない。
接触フィードバックの実装・校正・閉ループ安定性評価は後続作業とする。

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
    └─ log playback / media export

log_platform
    ├─ ROS bag ingestion
    ├─ topic extraction / time synchronization
    ├─ local playback data storage
    └─ MP4 / GIF / PNG / JPEG export

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

## 5.9.1 実験ログの可視化・メディア出力基盤

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
Local time-synchronized playback data
          │
     ┌────┴────┐
     ↓         ↓
3D motion   Time-series plots
     └────┬────┘
          ↓
Time-range MP4 / GIF
Current-frame PNG / JPEG
```

最低限、次のデータを再生対象とする。

- DRAGONのbase pose、内部関節角、目標形態
- Dracomancerの人体関節角とHuman desired shape
- 補正後のfeasible morphology
- \(f_{min}\)、\(\tau_{min}\)、推力飽和、障害物距離
- flight state、制約違反、gate rejection、failsafe等のイベント

ブラウザ表示は、URDF等から構成した3Dモデル、時系列グラフ、イベント一覧を同じ再生時刻へ同期する。再生、一時停止、任意時刻へのシーク、再生速度変更、表示topic選択を提供する。

ログ変換後のデータにはschema versionと元ログのhashを記録し、同じログから同じ表示を再生成できるようにする。動画は利用者が指定した開始・終了時刻の範囲をMP4またはGIFとして出力し、静止画は現在表示中の3DフレームをPNGまたはJPEGとして出力する。出力処理はブラウザ内で完結させ、クラウド保存、共有URL発行、公開範囲・失効管理は実装しない。非公開topic、操作者の個人情報、カメラ映像、位置情報が画面へ含まれる場合は、メディア出力前に利用者が内容を確認する。

大容量bagをブラウザへ直接読み込ませることを前提とせず、ROS側サーバまたはローカル変換ツールで必要topicを抽出・軽量化する。変換済み再生データはロボットPCのローカル領域へ保存し、元bagは変換後に削除することを既定とする。最大ファイルサイズ、サンプリング上限、元bag保持の要否はlaunch引数で設定できるようにする。

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

[10] ROBOTIS, “XM430-W350,” *DYNAMIXEL e-Manual*.
https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
