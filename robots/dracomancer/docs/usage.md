# 起動と検証

## launch一覧

| ファイル | 用途 |
|---|---|
| `bridge.launch` | Dracomancer用Spinal rosserial bridge。左右のサーボ・Joystickを公開し、各Joystickの`raw`から`joy`を生成する。 |
| `bringup.launch` | 通常のシミュレーション・実験用。Mk-IIのモデルとバックエンドに、DRAGONの位置・姿勢操作、内部関節角操作、力覚提示を組み合わせる。 |
| `haptics.launch` | 実機サーボで力覚提示を行う。 `simulation:=true`でDRAGONの壁接触を観測できる。 |
| `pose.launch` | 既存のDRAGONとJoystick入力に接続して、位置・姿勢を操作する。 |
| `shape.launch` | 既存のDracomancerとDRAGONに接続し、DRAGONの内部関節角操作と可行性判定を起動する。 |
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
[CAD基準URDF](../urdf/dracomancer.urdf)を読み込み、肩が上になるようモデル全体を90度回転して表示する。
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
roslaunch dracomancer bringup.launch enable_position_control:=true enable_joint_angle_control:=true \
  shape_config:=/absolute/path/to/shape.yaml
# 操作を開始する条件がそろってから有効にする
rostopic pub -1 /dracomancer/teleop_mode std_msgs/Bool 'data: true'
```

統合起動で力覚提示も使う場合は`enable_haptics:=true`を追加する。実機では[校正とHOLD操作](hardware.md#実機の力覚提示)が必要。
従来の`start_control:=true`は、操作指令が無効な状態でも制御ノードを起動する。

稼働中のDracomancerとDRAGONに操作を追加する場合は、必要なlaunchを選ぶ。

```bash
# 位置・姿勢操作
roslaunch dracomancer pose.launch shape_config:=/absolute/path/to/shape.yaml
# 内部関節角の操作と可行性判定
roslaunch dracomancer shape.launch shape_config:=/absolute/path/to/shape.yaml
```

位置・姿勢操作にはHover、新鮮なオドメトリ、操作者の世界座標（`navigation.operator_position`または外部計測）、
既定ではIMUが必要。片腕では押下で並進と回転を切り替え、両腕では右が並進・左が回転を担当する。
DRAGONの内部関節角の操作には、校正済みの関節計測と、体幹点・軸合わせを含む[形状設定](shape.md#通常起動と設定)が必要。

既存workspaceを更新する場合は、削除済みパッケージ・メッセージ・プラグインの生成物を
Catkinのclean機能で整理してからビルドする。
旧構成のPython実行ファイルが残っている場合も、`catkin clean dracomancer`で整理してからビルドする。

## 自動bag記録

`bringup.launch`は既定で全ROSトピックを記録する。保存先はDracomancerパッケージの`rosbags/`で、
ファイル名は起動PCの日時を使った`YYYYMMDD_HHMMSS_bringup.bag`。同じ名前がある場合は末尾に連番が付く。
Ctrl-Cでlaunchを終了すると、記録中の`.bag.active`が`.bag`になる。

```bash
roslaunch dracomancer bringup.launch bag_name:=shape_test bag_dir:=/path/to/rosbags
roslaunch dracomancer bringup.launch record_bag:=false
```

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

`ctest -R spinal_current`はSpinal側の電流セッション・期限を確認する。
修論対応で`ShapeFeasibility`サービス定義を拡張したため、`catkin build spinal dracomancer`を行い、
依存クライアントも新しい生成済みサービス型で再起動する。
Spinalの14軸拡張を取り消したため、拡張済みイメージを使用している実機にはMCU側の再ビルド・書き込みも必要。
独立した左右7軸の実機構成に必要な開発は[残作業](roadmap.md#第2優先・補助開発)を参照する。
未確認事項と次に必要な評価は[残作業](roadmap.md#再検証が必要な範囲)にまとめる。

## ROSサービス定義

`srv/ShapeFeasibility.srv`は、DRAGONの候補関節角が飛行可能性のゲートを満たすか評価するサービスの型定義。
`shape_feasibility_node`が既定で`/dragon/shape_feasibility/check_shape`を提供し、
`control_joint_angle.py`が候補形態と速度制限後の発行候補を検査する。

| 方向 | 主な内容 |
|---|---|
| リクエスト | 関節名と候補角、必要に応じて接触レンチ、接触点、再評価で使う固定済みのジンバル角と姿勢。 |
| レスポンス | 力・トルクの余裕半径、釣合い判定`stability_ok`、計算の有効性`valid`、再評価用のジンバル角・姿勢と座標情報。 |
