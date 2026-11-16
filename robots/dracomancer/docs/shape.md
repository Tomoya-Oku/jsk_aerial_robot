# 形状マッピング：実装・検証記録

2026-11-17のパッケージ統合後はMk-II専用・名前空間/dracomancerを使用する。
実機はmeasured_joint_statesを入力とする。入口と計測条件は[操作手順](mkii.md)を参照。
以下の2026-11-16の結果は統合前の検証記録であり、今回の再検証は[統合記録](integration.md)を参照。

数理仕様・受入条件は[実装指示書](dracomancer_shape_retargeting_codex.md)、
研究設計は[ソフトウェア設計](design-spec/05-software.md)を参照する。
本書は起動方法、実装との対応、検証結果を記録する。

## 起動

```bash
catkin build dracomancer --no-deps
source devel/setup.bash

# Mk-IIの関節スライダ + DRAGON Gazebo + 両機体と中心線のRViz
roslaunch dracomancer shape_demo.launch

# 同じ肩・肘入力を自動で繰り返す（スライダを同時に操作しない）
roslaunch dracomancer shape_demo.launch motion:=true

# 画面なしの検証
roslaunch dracomancer shape_demo.launch motion:=true gui:=false rviz:=false
```

既存Gazebo/DRAGONを停止してから起動する。既存のDRAGON起動が使うモデル名・名前空間を再利用するため、
同じmasterにDRAGONを二重起動しない。デモは地上の形態操作で、離陸操作も偽のHOVER状態配信も行わない。
デモだけはHover条件を明示的に解除し、既存のsimulation用preflight関節制御で動作させる。
通常の`teleoperation.launch`ではHover条件を維持する。

RVizは、Mk-II、Gazebo実測のDRAGON、物理中心線、別位置に重ねた正規化中心線を同時表示する。
水色が人体、橙色がDRAGON。点は物理端点または等弧長サンプルである。
Mk-IIは`dracomancer`のCADモデル・viewer・パネルを使用し、旧Mk-Iモデルには変換しない。
手動時は右腕`right_joint_1..7`を動かす。左腕は本マッピングの入力ではない。

## 設定と制御経路

| ファイル | 役割 |
|---|---|
| `scripts/control/shape_mapping.py` | ROS非依存URDF FK、中心線、制約付き数値最適化、速度制限 |
| `scripts/control/shape_runtime.py` | 入力監視、既存ゲート接続、周期内キャッシュ、診断・Marker |
| `scripts/control/control_joint_angle.py` | 既存の単一指令発行ノード。`shape`の明示分岐と速度制限 |
| `config/control/shape.yaml` | 代表点・固定軸合わせ・最適化設定 |
| `launch/shape_demo.launch` / `config/control/shape.rviz` | 地上シミュレーション・可視化 |
| `scripts/control/shape_input.py` | 任意の再現用Mk-II入力。DRAGON指令は発行しない |
| `tests/test_shape.py` / `tests/demo_probe.py` | 単体検証 / 起動済みGazeboの実測検証 |

通常設定の`torso_point: []`ではshapeモードを有効化しない。利用時は`shape_config`で設定ファイルを指定する。
既存DRAGON・Mk-IIを個別起動する場合は、校正した設定を用いて
`roslaunch dracomancer teleoperation.launch device_ns:=dracomancer mapping_mode:=shape shape_config:=/path/to/shape.yaml`
とする（通常のHover条件を適用）。
デモでは指示者が承認した仮想体幹点`base_link`原点を、launch引数`torso_point`で明示的に設定する。
肩・肘・手首は設定された右腕CADフレーム原点、手先は手先リンクのCAD重心を使う。
最後の手首回転を中心線に反映させるため、手先点は回転軸原点から離す。
これらとidentityの固定軸合わせはシミュレーション用の幾何代理点・初期校正であり、人体の解剖学的測定値ではない。
URDFを変更した場合は代表点・`dragon_tail`を再確認する。

人体は固定`base_link`基準で計算し、肩の相対回転を保持する。
DRAGONは実際のURDF関節列を正方向に計算後、Link4尾端へ逆変換して反転する。
4本のリンクと3本のPitch–Yaw間スペーサを含む8端点を保持する。
旧`shape_task/dragon_fk.py`はスペーサの折れ角を省略した5点近似であり、新モードと同一モデルではない。
既存`E_s`の意味・結果互換性を維持するため旧実験FKは変更しない。

`q_prev`は最後の発行指令、`q_rest`は`shape/rest`（未指定ならstartup pose）。
関節角・速度上限は読み込んだDRAGON URDFを使用する。
既定で最適化10Hz・最大1反復・時間予算20ms、指令ループ40Hz。
更新間には直前の最適化結果を再利用し、不正入力時には破棄する。
初期姿勢は新鮮な実測関節角と既存ゲートで検証するまで指令を抑止し、Gazebo初期化との競合を避ける。

新モードは`hold`ゲートのみ使用する。候補を検査し、同一候補の検証結果を評価周期内で再利用する。
速度制限後の各発行形態は改めて検査する。サービス失敗・不正入力・時間超過では最後の発行指令を保持する。
初期校正が無い場合は指令発行を開始しない。Link4アンカーやbase pose操作へは接続しない。

Mk-Iの`distal`・`geometric`・`joint_pairing`は削除し、既定・対応モードを`shape`に統一した。
shapeの発行指令に、既存`max_step`に加えて経過ROS時間による`max_joint_velocity`（既定0.1 rad/s）を適用する。
shapeではURDF速度上限との小さい方を使う。停止中・時計巻戻り・遅延分の追い付き動作は加算しない。


## 診断と記録

デモの名前空間は`/dracomancer`。通常teleoperationも同じ名前空間を使用する。

| トピック | 内容 |
|---|---|
| `/dracomancer/candidate/joint_target` | ゲート前の`q_ref`（既存JointState） |
| `/dragon/joints_ctrl` | 最終発行指令`q_command`（既存JointState） |
| `/dragon/joint_states` | Gazeboの実関節フィードバック |
| `/dracomancer/shape/rmse` | 人体対発行指令の無次元・重み付き位置RMSE |
| `/dracomancer/shape/diagnostics` | JSON：q_ref、q_command、候補/指令RMSE、seconds、iterations、ok、status、gate、updated、control_seconds |
| `/dracomancer/shape/markers` | 物理端点・等弧長サンプルのMarkerArray |
| `/dracomancer/shape_control_error` | 従来どおり関節候補−発行指令の6値 |

`seconds`は直前の最適化更新（人体FKを含む）の処理時間。`updated=true`だけを集計する。
`control_seconds`はマッピング・ゲート・最終指令検査までの時間で、診断/Marker配信とROS sleepを含まない。
入力異常後のRMSE/Markerは最後の有効入力を参照するため、`status`・`ok`と併せて読む。

```bash
rosbag record -O shape.bag /dracomancer/measured_joint_states /dragon/joint_states \
  /dragon/joints_ctrl /dracomancer/candidate/joint_target \
  /dracomancer/shape/diagnostics /dracomancer/shape/rmse \
  /dracomancer/shape_control_error /dragon/flight_state
```

`shape_task`の`E_s`とは別指標であり、既存CSVの列の意味は変更しない。
CAD関節角と旧Mk-I解剖学的関節角は同じではないため、固定対応比較手法のMk-II向け再実装・校正は別途必要。

## 検証

```bash
OPENBLAS_NUM_THREADS=1 python3 -B robots/dracomancer/tests/test_shape.py -v
# shape_demo.launch motion:=true起動後、別端末で実行
python3 -B robots/dracomancer/tests/demo_probe.py --output /tmp/shape_probe.json
```

2026-11-16、Intel Core Ultra 7 165H、ROS One / Gazebo Classic、NumPy 1.26.4で検証。
計測の生データと画面は一時生成物として`/tmp/dracomancer-shape/`へ保存し、Gitには追加しない。

- 単体テスト26件：関節名順、欠損/NaN/Inf、体幹未設定、固定取付変換、肩/最終手首回転、等弧長・相似形状、
  URDF/KDL FK、物理スペーサ・逆順、到達可能/不能形状、境界での停滞、正則化・連続性、時間超過保持、
  経過時間・時計巻戻りの速度制限、Hover・入力準備・サービス不正結果などを確認。
- `catkin build dracomancer --no-deps`成功（既存distutils非推奨警告あり）。
- `develop`基点`abbc6ddb`の既存マッピングと3モード×100入力の出力を比較し、絶対誤差`1e-12`以下で一致。
  速度制限を適用する前のマッピング出力の比較である。
- `shape_task.launch dryrun:=true`で開始・停止サービス成功、擬似HOVER状態を確認。
- Gazebo実行中に入力ノード停止、NaN、関節名欠損、未来時刻、予測サービス停止を注入し、
  各条件で最終発行指令の変化量が0であることを確認。
- GazeboでMk-IIの肩・肘入力に伴うDRAGON実関節変位を確認。
  RVizで両モデルと物理/正規化中心線を確認した。
- 代表入力における人体中心線形状ヤコビアンの特異値（未重み付け、`60×7`）は、CADゼロ姿勢で
  `[0.84235, 0.77200, 0.14051, 0.06573, 0.05345, 0.01800, 0.000329]`、全軸0.3radで
  `[0.80751, 0.59518, 0.14582, 0.11835, 0.05362, 0.02575, 0.001396]`。
  CADの取付オフセット・代理点に依存する数値ランクで、人体7DoFの可逆な6DoF転写を意味しない。
  理想的な中心線に現れない純粋な軸回旋・手先姿勢情報は復元できない。

最終の画面なし自動入力試験（15秒、601指令、最適化更新139回、専用ROS/Gazeboポート）の計測結果：

| 処理時間 [ms] | 平均 | 95パーセンタイル | 最大 |
|---|---:|---:|---:|
| 人体FK＋数値最適化 | 4.48 | 6.59 | 8.33 |
| マッピング＋ゲート＋最終検査 | 4.45 | 11.53 | 15.47 |

最適化時間超過0回。後者は最適化を行わない40Hz周期も含み、診断/Marker配信は含まない。
この計測条件では25msの処理目標内だったが、他プロセスや描画を含む負荷の高い試行では超過も発生した。
上記はハードリアルタイム性の保証ではない。

発行角速度の最大値は0.09231rad/s（上限0.1rad/s）。
最大の関節指令変位幅は0.4171rad、Gazebo実関節の変位幅は0.4639radだった。
実際の指令送信元は`/dracomancer_control_joint_angle`だけだった。
URDF FKとrobot_state_publisherの実TF位置比較は、人体24点で最大`3.62e-16m`、
DRAGON6点で最大`5.99e-16m`（許容`1e-8m`）。
単体ではCADゼロ姿勢および全軸`0,0.13,...,0.78rad`、実TF比較では自動入力軌跡上の姿勢を使用した。
Gazeboの実測関節角には小さな境界超過があり、比較用FKでは実測値をそのまま記述する。
人体入力・最適化・発行指令の関節境界チェックは有効なままである。

## 限界・未確認事項

- 速度上限は発行目標の角速度制限。実サーボの追従誤差・外力による実関節運動の上限保証ではない。
- 実機用の仮想体幹点、人体代表点、固定軸合わせ、安全速度・推力余裕閾値は未校正。
- 候補と発行形態を検査しても、経路内の全中間点、動的安定性、接触、衝突回避は保証しない。
- 既存の同期feasibility RPCは接続待ち・計算時間を含み、40Hzのハードリアルタイム保証は無い。
  時間予算超過の最適化更新は採用せず保持する。診断から更新/周期時間を区別して評価する。
- 既存DRAGON navigatorも`joints_ctrl`をadvertiseするが、地上デモで実際に指令送信したのは形態制御ノードのみ。
  離着陸・別デモ・キーボード制御を並行実行しない。
- 地上でのPitch変形は地面との接触で追従が悪化することがある。飛行中の追従性能は未確認。
- 既存ライブfc監視は地上で値が配信されず`missing_inradius`になる場合がある。
  shapeの採否には独立した既存予測サービスの`valid`・`stability_ok`と閾値を使う。
- Mk-IIでの校正済みA/B比較、実機、飛行中の形態変化、接触作業・力覚提示は今回の検証対象外。
- 力覚ブランチ統合後もshapeの接触力覚は無効。起動排他と実機出力の条件は
  [ソフトウェア設計5.7.3](design-spec/05-software.md#573-現在の対応範囲)を参照する。
- 繰返し起動の途中で、既存`attitude_controller`のロード時に`gzserver`が一度exit 139で終了した。
  根因は未特定。画面なしの再起動では統合テストを実行できたが、起動安定性の保証は未確認。
