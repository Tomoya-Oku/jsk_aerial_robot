# DRAGONの形状操作

形状写像の数理・制御方針は[ソフトウェア設計](software.md#全体形状写像b)、
実験条件は[評価計画](evaluation.md)、実機入力は[校正手順](hardware.md)を参照する。

## 地上デモ

Catkin workspaceでビルド・sourceしてから実行する。

```bash
roslaunch dracomancer shape_demo.launch
# 同じ肩・肘入力を繰り返す。スライダを同時操作しない。
roslaunch dracomancer shape_demo.launch motion:=true
# 画面なし
roslaunch dracomancer shape_demo.launch motion:=true gui:=false rviz:=false
```

Mk-II viewerとDRAGON Gazeboを起動する。離陸も偽のHOVER配信も行わない。
デモだけはHover条件を外し、既存のsimulation用preflight関節制御で地上動作させる。
通常の`teleoperation.launch`ではHover条件を維持する。
他のDRAGON、Gazebo、`/dragon/joints_ctrl`指令ノードを重複起動しない。

手動入力は右腕`right_joint_1..7`。左腕は写像入力に使用しない。
RVizには両モデル、物理端点、等弧長点、重ねた正規化中心線を表示する。水色が人体、橙色がDRAGON。

## 通常起動と設定

Mk-IIとDRAGONを別々に起動する場合は、校正した設定を指定する。

```bash
roslaunch dracomancer teleoperation.launch shape_config:=/absolute/path/to/shape.yaml
# 条件がそろってから形状操作を有効化する
rostopic pub -1 /dracomancer/teleop_mode std_msgs/Bool 'data: true'
```

設定元は`config/control/shape.yaml`。標準の`torso_point: []`では写像を有効化しない。
体幹点、人体代表点、固定軸合わせを実機に合わせて設定する。
`shape_demo.launch`と`contact.launch`は、シミュレーション用に`base_link`原点の体幹代理点を明示する。
CADフレーム原点・手先重心とidentityの軸合わせも代理値で、解剖学的な実測値ではない。
最後の手首回転を反映するため、手先点を回転軸原点から離す。
URDFを変更したら人体代表点と`dragon_tail`の整合を確認する。

標準入力は`/dracomancer/measured_joint_states`、人体モデルは`/dracomancer/robot_description`。
形態指令は`control_joint_angle.py`だけが発行する。別ノードから同じtopicへ指令しない。
対応モードは`shape`、feasibility gateは`hold`のみ。Link4アンカーや腕全体からの機体移動へ接続しない。
`teleop_mode=false`は最後の指令を保持する。現在のclutch解除は人体基準姿勢の再設定を行わない。

## 実装の入口

| ファイル | 役割 |
|---|---|
| `scripts/control/shape_mapping.py` | URDF FK、中心線、最適化、速度制限 |
| `scripts/control/shape_runtime.py` | 入力・モデル確認、ゲート、周期内キャッシュ、診断・Marker |
| `scripts/control/control_joint_angle.py` | 単一の指令発行ノード、モード・Hover条件 |
| `scripts/control/shape_input.py` | デモの再現用入力。DRAGONへは指令しない |
| `config/control/shape.yaml` | 代表点・軸合わせ・最適化設定 |
| `tests/test_shape.py`、`tests/demo_probe.py` | 単体検証、起動中のGazebo・TFとの照合 |

初期化は新鮮な機体実測姿勢・入力・設定・可行性を確認するまで待機する。
最適化失敗、入力欠損・古い時刻・未来時刻、不正値、時間超過、サービス障害では指令を進めない。
候補を検査し、速度制限後の発行形態も再検査する。検査済み候補の再利用は同一候補に限定する。

## 診断とログ

| Topic | 内容 |
|---|---|
| `/dracomancer/candidate/joint_target` | ゲート前の候補`q_ref` |
| `/dragon/joints_ctrl` | 発行指令`q_command` |
| `/dragon/joint_states` | 機体の実関節フィードバック |
| `/dracomancer/shape/rmse` | 人体対発行指令の正規化・重み付き位置RMSE |
| `/dracomancer/shape/diagnostics` | JSON：候補・指令、誤差、処理時間、反復、ok・status・gate・updated |
| `/dracomancer/shape/markers` | 物理端点と正規化中心線のMarkerArray |
| `/dracomancer/shape_control_error` | 関節候補−発行指令の6値。形状RMSEとは別指標 |

`seconds`は人体FKを含む直前の最適化更新時間。`updated=true`を対象に集計する。
`control_seconds`は写像・ゲート・最終検査までで、診断・Marker配信とROS sleepを含まない。
入力異常後のRMSE・Markerが最後の有効入力を参照する場合があるため、`status`・`ok`と併せて読む。

```bash
rosbag record -O shape.bag /dracomancer/measured_joint_states /dragon/joint_states \
  /dragon/joints_ctrl /dracomancer/candidate/joint_target \
  /dracomancer/shape/diagnostics /dracomancer/shape/rmse \
  /dracomancer/shape_control_error /dragon/flight_state
```

起動中のデモを検証する場合は、ソース取得先で`tests/demo_probe.py`を実行する。
検証コマンドの一覧は[操作手順](usage.md#再現できる検証)を参照する。

## 評価時の注意

`shape_task/dragon_fk.py`は目標形態との`E_s`・シャドウ用の簡易FKで、スペーサを含む本写像のFKと同一ではない。
既存CSVの意味を変えず、Human–Robot形状RMSEとは区別する。
中心線に現れない軸回旋・手先姿勢は復元できず、CAD関節と人体関節が同じ座標であるとも扱わない。
速度制限は発行目標の制限で、実関節速度・経路中間点・動的安定性・衝突回避の保証ではない。
同期feasibility RPCと描画・他プロセスの負荷を含む周期時間は実環境で測定する。
再検証・実機校正・力覚接続の残作業は[roadmap.md](roadmap.md)を参照する。
