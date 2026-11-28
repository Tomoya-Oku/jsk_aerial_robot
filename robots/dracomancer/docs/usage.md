# 起動と検証

## launch一覧

| ファイル | 用途 |
|---|---|
| `bridge.launch` | Dracomancer用Spinal rosserial bridge。左右のサーボ・Joystickを公開し、各Joystickの`raw`から`joy`を生成する。 |
| `bringup.launch` | 通常のシミュレーション・実験用。Mk-IIのモデル、バックエンド、DRAGONの内部関節角・位置・姿勢操作をまとめて起動できる。 |
| `haptics.launch` | 実機サーボで力覚提示を行う。 `simulation:=true`でDRAGONの壁接触を観測できる。 |
| `position_attitude.launch` | DRAGONの位置・姿勢操作だけを起動する。既存のDRAGONとJoystick入力に接続する。 |
| `shape_demo.launch` | DRAGONを離陸させず、形態写像・両モデルの姿勢をGazeboとRVizで比較する。 |

## 設定ファイル一覧

パスは `config/` からの相対パス。

| ファイル | 用途 |
|---|---|
| `model/joints.yaml` | 左右14関節のROS名と並び順。シミュレーションのトルク上限・指令タイムアウトも設定する。 |
| `model/joint_mapping.csv` | URDFの各関節と親子リンク・STL・回転軸の対応表。 |
| `model/cad.json` | CADの版、リンク・関節数、質量、STLの外接寸法、URDF・STLのハッシュを記録し、モデルテストが参照する。 |
| `device/hardware.yaml` | 関節とDYNAMIXEL物理ID・Spinal scan indexの対応、零点・方向、電流とトルクの換算・上限を設定するテンプレート。配布時は未校正。 |
| `device/joystick_calibration.yaml` | 右Joystickの保存済み校正。軸の中心・上下限・デッドゾーン・反転を指定する。 |
| `simulation/controllers.yaml` | Gazeboの各関節の位置・トルク・状態配信コントローラと、位置制御のPID設定。 |
| `simulation/contact.yaml` | Gazeboの接触用の壁の位置・寸法・摩擦係数。 |
| `control/shape.yaml` | 腕からDRAGONへの形態写像、位置・姿勢操作、力覚、接触レンチの設定。 |
| `visualization/mkii.rviz` | 標準のDracomancerモデルを表示するRViz設定。 |
| `visualization/cad.rviz` | `assembly:=true`でCADアセンブリを表示するRViz設定。 |
| `visualization/observe.rviz` | `backend:=preview`で未校正の実機を起動時姿勢基準で表示するRViz設定。 |
| `visualization/shape.rviz` | DracomancerとDRAGONのモデル、形態比較マーカーを表示する地上デモ用RViz設定。 |
| `visualization/plotjuggler.xml` | DRAGONの力・トルク余裕について、観測値・候補値・しきい値を比較するPlotJugglerレイアウト。 |

## ビルドと基本操作

ROS・外部依存は[リポジトリREADME](../../../README.md)に従う。以下はCatkin workspace直下で実行する。
`shape_feasibility_node`はDRAGONのモデルヘッダー経由でNLoptのC++ヘッダーを使うため、
Dracomancer自身も`nlopt`への依存とCMakeの検索・リンク設定を宣言する。

```bash
catkin build spinal dracomancer
source devel/setup.bash

# 標準はviewer。実機へ接続せずCADモデルと角度スライダを表示する。
roslaunch dracomancer bringup.launch
# Mk-II自身の物理シミュレーションと角度・トルク操作
roslaunch dracomancer bringup.launch backend:=gazebo
```

`rviz:=false panel:=false`で表示を省略できる。`backend:=preview`は未校正の実機を読み取り専用で表示する。
標準名前空間は`/dracomancer`。右腕・左腕のCAD関節名と通信仕様は[デバイス設計](device.md)を参照する。

標準のviewer・Gazeboと`assembly:=true`は、2026年11月27日保存のSolidWorksアセンブリから
マネキンを除いた両腕モデルを使う。全関節角がゼロのとき、関節姿勢はCADの保存状態と一致する。
`roslaunch dracomancer bringup.launch assembly:=true mount_height:=0.0`では
[CAD基準URDF](../urdf/cad.urdf)を読み込み、肩が上になるようモデル全体を90度回転して表示する。
別ウィンドウの`Angles [rad]`タブで、左右14関節を個別に操作できる。
同番号の左右関節に同じ角度を入れると、それぞれのCAD基準姿勢から鏡像方向へ回転する。
DRAGONと組み合わせる場合は[地上デモ](shape.md#地上デモ)を参照。

実機は`backend:=hardware config:=/absolute/path/to/hardware.yaml`を指定し、[実機手順](hardware.md)に従う。
bridgeも起動する場合だけ`connect_fc:=true fc_serial_port:=/dev/ttyUSB0`を加える。
通常は既存のSpinal rosserial bridgeへ接続し、ファームウェアを書き込まない。

形態入力は`measured_joint_states`。実機の`joint_states`は未計測関節の表示用q=0を含む。
DRAGONを起動した環境で、下記のようにDracomancer側の制御をまとめて起動する。
実機では`backend:=hardware`と校正済みの`config`を追加する。

```bash
roslaunch dracomancer bringup.launch enable_position_control:=true \
  shape_config:=/absolute/path/to/shape.yaml
# 操作を開始する条件がそろってから有効にする
rostopic pub -1 /dracomancer/teleop_mode std_msgs/Bool 'data: true'
```

位置・姿勢操作だけを既存のDRAGONとbridgeへ追加する場合は、同じ`shape_config`を指定して
`roslaunch dracomancer position_attitude.launch shape_config:=/absolute/path/to/shape.yaml`を起動する。
両入口とも操作にはHover、新鮮なオドメトリ、操作者の世界座標（`navigation.operator_position`または外部計測）、
既定ではIMUが必要。片腕では押下で並進と回転を切り替え、両腕では右が並進・左が回転を担当する。
DRAGONの内部関節角の操作には、体幹点と軸合わせを含む[形状設定](shape.md#通常起動と設定)も必要。

既存workspaceを更新する場合は、削除済みパッケージ・メッセージ・プラグインの生成物を
Catkinのclean機能で整理してからビルドする。

## 壁接触の観測

```bash
roslaunch dracomancer haptics.launch simulation:=true
```

Mk-II viewer、DRAGON Gazebo、固定壁を起動し、Link1先端面の接触を`/dragon/contact/wrench`へ配信する。
壁の配置・寸法・摩擦は`config/simulation/contact.yaml`で設定する。実機へ接続せず、既定では接触力覚を出力しない。
接触レンチを含む静的釣合いを判定し、`enable_haptics:=true`を明示した場合だけ力覚計算を追加する。
形態操作にはシミュレーション用の体幹代理点を明示し、通常のHover・可行性条件を使う。
移動は独立したJoystick速度入力。本起動だけでは離陸せず、必要な離着陸は既存DRAGONの手順で操作者が行う。
Mk-II単体Gazebo、DRAGONの別Gazebo、重複する形態指令ノードを同じmasterへ同時起動しない。
実機サーボによる力覚提示の手順は[実機の力覚提示](hardware.md#実機の力覚提示)を参照する。

## 再現できる検証

ソースを取得したworkspaceで、変更内容に応じて次を使う。
テストは`test/`へ集約し、Mk-II固有のモデル・直結制御・GUI検証は`test/mkii/`に置く。

```bash
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/mkii/model.py
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/mkii/current.py
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/test_runtime.py
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/test_output.py
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/test_contact.py
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/test_haptics.py
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/test_theory.py
python3 -B src/jsk_aerial_robot/robots/dracomancer/test/test_shape.py -v
```

`test_runtime.py`はROSインターフェースを模擬した回帰テスト。`test_shape.py`にはROS・PyKDL依存の検証を含む。
`ctest -R dracomancer_wrench`は固定Qの符号付き余裕と共通配分、`ctest -R spinal_current`は電流セッション・期限を確認する。
修論対応で`ShapeFeasibility`サービス定義を拡張したため、`catkin build spinal dracomancer`を行い、
依存クライアントも新しい生成済みサービス型で再起動する。
Spinalの14軸拡張を取り消したため、拡張済みイメージを使用している実機にはMCU側の再ビルド・書き込みも必要。
独立した左右7軸の実機構成に必要な開発は[残作業](roadmap.md#第2優先・補助開発)を参照する。
実際のROS・Qt・Gazeboでは、実機のmasterを再利用せず隔離環境で次を実行する。

```bash
bash src/jsk_aerial_robot/robots/dracomancer/test/mkii/run.sh viewer
bash src/jsk_aerial_robot/robots/dracomancer/test/mkii/run.sh gazebo
bash src/jsk_aerial_robot/robots/dracomancer/test/mkii/run.sh direct
bash src/jsk_aerial_robot/robots/dracomancer/test/gazebo.sh
```

`direct`試験は偽Spinalを使い、実サーボへ接続しない。
単体試験はCAD整合・数値処理・入力監視の確認で、実際の通信断、物理停止、飛行安定性の検証とは区別する。
未確認事項と次に必要な評価は[残作業](roadmap.md#再検証が必要な範囲)にまとめる。
