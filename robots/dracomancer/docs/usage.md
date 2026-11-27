# 起動と検証

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

`rviz:=false panel:=false`で表示を省略できる。`mkii.launch`は同じ引数を持つ入口。
個別の`view.launch`・`gazebo.launch`・`hardware.launch`も利用できる。
標準名前空間は`/dracomancer`。右腕・左腕のCAD関節名と通信仕様は[デバイス設計](device.md)を参照する。

`roslaunch dracomancer cad.launch`で、2026年11月27日保存のSolidWorksアセンブリから
マネキンを除いた両腕モデルを表示する。[表示用URDF](../urdf/assembly.urdf)の全関節角が
ゼロのとき、関節姿勢はCADの保存状態と一致する。RVizでは肩が上になるよう
モデル全体を90度回転して表示する。表示用の名前空間は`/dracomancer_cad`。

実機は`backend:=hardware config:=/absolute/path/to/hardware.yaml`を指定し、[実機手順](hardware.md)に従う。
bridgeも起動する場合だけ`connect_fc:=true fc_serial_port:=/dev/ttyUSB0`を加える。
通常は既存のSpinal rosserial bridgeへ接続し、ファームウェアを書き込まない。

全backendの形態入力は`measured_joint_states`。実機の`joint_states`は未計測関節の表示用q=0を含む。
`bringup.launch teleop_mode:=true shape_config:=/absolute/path/to/shape.yaml`で形状操作も起動できるが、
体幹点・固定軸合わせを設定しなければ形態指令を開始しない。[形状操作](shape.md)で設定・ログを確認する。

既存workspaceを更新する場合は、削除済みパッケージ・メッセージ・プラグインの生成物を
Catkinのclean機能で整理してからビルドする。

## 壁接触の観測

```bash
roslaunch dracomancer contact.launch
```

Mk-II viewer、DRAGON Gazebo、固定壁を起動し、Link1先端面の接触を`/dragon/contact/wrench`へ配信する。
壁の配置・寸法・摩擦は`config/contact.yaml`で設定する。実機へ接続せず、既定では接触力覚を出力しない。
接触レンチを含む静的釣合いを判定し、`enable_haptics:=true`を明示した場合だけ力覚計算を追加する。
形態操作にはシミュレーション用の体幹代理点を明示し、通常のHover・可行性条件を使う。
移動は独立したJoystick速度入力。本起動だけでは離陸せず、必要な離着陸は既存DRAGONの手順で操作者が行う。
Mk-II単体Gazebo、DRAGONの別Gazebo、重複する形態指令ノードを同じmasterへ同時起動しない。

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
