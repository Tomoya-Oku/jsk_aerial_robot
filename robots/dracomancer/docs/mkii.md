# Mk-IIのURDF・Gazebo・RViz

Mk-Iの既存 `bringup.launch`、関節名、設定は維持する。
Mk-IIのCADモデルと操作ノードは同じリポジトリの `robots/dracomancer_mkii_ros/` に配置する。
研究仕様は [4章](design-spec/04-hardware.md) と [5章](design-spec/05-software.md) が正本。
ここでは起動・検証方法を扱う。

## ビルド

ROS 1環境で、このリポジトリを配置したCatkin workspaceから実行する。

```bash
catkin build spinal dracomancer_mkii_ros dracomancer
source devel/setup.bash
```

新しいPythonパッケージとSpinalのメッセージがあるため、launchのコピーだけでは不十分。
通常のdevelとinstallの双方にURDF、メッシュ、起動設定、操作ノードを導入する。

## 起動

```bash
# RViz＋角度ウィンドウ（既定）
roslaunch dracomancer mkii.launch
# Gazebo Classic＋RViz＋角度／トルクウィンドウ
roslaunch dracomancer mkii.launch backend:=gazebo
# 画面を開かずにシミュレーションする場合
roslaunch dracomancer mkii.launch backend:=gazebo gui:=false rviz:=false panel:=false
```

`roslaunch dracomancer_mkii_ros view.launch` / `gazebo.launch` も利用できる。
既定の名前空間は `/dracomancer_mkii`。Mk-Iの `/dracomancer` と区別する。
`robot_ns`、`mount_height`、`rviz`、`panel` を必要に応じて指定できる。
Gazebo Classic向けであり、ROS 2や新Gazebo Sim向けのlaunchではない。

AnglesタブはMk-Iと同じ `joint_state_publisher_gui` を再利用する。
Torqueタブは関節ごとに位置／effort controllerを排他的に切り替える。
Nmを指定してHOLDを押し続けると送信し、解放・ZERO・指令途絶でゼロにする。
ゼロトルクは重力に抗する保持ではなく、腕が落下することがある。
viewerのみではトルク操作は無効。

## モデルと実機

`urdf/cad.urdf` が基準のCAD由来URDF、`urdf/mkii.urdf.xacro` がGazebo用モデル。
どちらも同じパッケージの8種類のSTLを参照する。
Mannequinを除く15リンク、左右各7関節。world固定はシミュレーション用の追加要素。
連続関節とCAD保存姿勢のq=0は暫定で、実機の可動範囲・原点ではない。
PID、2Nm／2rad/s、減衰、箱近似collisionも仮設定で、実機の保証値ではない。

Spinal UART直結の操作と校正、対応ボード、復旧方法は
[実機手順](../../dracomancer_mkii_ros/docs/hardware.md) を参照。
既定の実機設定は未校正・空の対応表で駆動しない。
この導入ではファームウェア書込みや実機駆動を行わない。

## 検証

```bash
cd <workspace>/build/dracomancer_mkii_ros
ctest -R mkii_ --output-on-failure
cd <workspace>/build/spinal
ctest -R spinal_current --output-on-failure
# リポジトリのルートから。専用masterを起動し、実機には接続しない
bash robots/dracomancer_mkii_ros/test/run.sh viewer
bash robots/dracomancer_mkii_ros/test/run.sh gazebo
bash robots/dracomancer_mkii_ros/test/run.sh direct
```

`run.sh` は使用するローカルポートが空いていることを確認し、自分のROS/Gazeboだけを終了する。
結果は一時ディレクトリに保存し、そのパスを表示する。ビルド生成物やログはコミットしない。
modelテストはこのパッケージ内のCAD基準・メッシュhashを使い、別のエクスポータリポジトリを必要としない。

## 導入の取り消し

起動したroslaunchを終了する。Mk-II用パッケージと `mkii.launch` を導入前へ戻してCatkinを再ビルドする。
既存Mk-Iのbringupは変更しない。実機へ書き込んだ後の復旧は別途元ファームウェアとサーボ設定が必要。

### WSLでの時刻とペーシング

2026-11-16の検証環境では、単調時計で約1秒の間に実時間時計が約7秒進む現象を観測した。
Gazeboのフィードバックが数秒止まる場合があり、通常速度での連続動作は未確認。
機能検証には、テスト限定で実時間ペーシングを外す設定を使用できる。

```bash
MKII_TEST_UNPACED=1 bash robots/dracomancer_mkii_ros/test/run.sh gazebo
```

これはGazeboのmax_update_rateだけを0へ変更し、時間刻み・重力・ODE設定を維持する。
通常launchの既定値と、指令・フィードバックの期限監視は変更しない。

## 確認結果（2026-11-16）

- Spinal・Dracomancer・dracomancer_mkii_rosのCatkinビルド成功。
- CTestのmkii_current・mkii_model・spinal_currentが成功。
- Mk-IのbringupとMk-IIのviewer/gazebo/hardwareのlaunch解析が成功。不正backendは拒否。
- install領域からPythonパッケージ、URDF、8種類のSTL、手順書を解決できることを確認。
- viewerで14関節の状態と角度反映を確認。不正値とトルク操作を拒否。
- 偽SpinalとQtで準備時OFF、Nm指令、解放時OFF、期限切れ、遅延ARM取消、校正ゲート、通常rosrun起動を確認。
- GazeboはMKII_TEST_UNPACED=1で位置追従・effort切替・排他制御・一時停止中の期限監視・Qt操作を確認。
- 実機書込み・駆動、RVizの目視確認、通常速度でのWSL連続動作は未確認。

検証は専用のROS masterと偽ハードウェアを使用し、実機へ接続していない。
Windows上のソースを参照する隔離Catkin workspaceでビルドし、既存workspaceは変更していない。
設計仕様4章・5章が定める実機検証と校正を完了したものではない。
