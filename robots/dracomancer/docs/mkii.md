# Mk-IIの起動と検証

モデル・GUI・直結制御は`dracomancer`パッケージに統合した。
仕様の正本は[ハードウェア設計](design-spec/04-hardware.md)と[ソフトウェア設計](design-spec/05-software.md)。

## 起動

```bash
catkin build spinal dracomancer
source devel/setup.bash
roslaunch dracomancer bringup.launch
roslaunch dracomancer bringup.launch backend:=gazebo
roslaunch dracomancer bringup.launch backend:=hardware config:=/absolute/path/to/hardware.yaml
```

標準はviewer。`rviz:=false panel:=false`で表示を省略できる。
`mkii.launch`は`bringup.launch`と同じ入口として残す。個別の`view.launch`・`gazebo.launch`・`hardware.launch`も利用可能。
全入口の標準名前空間は`/dracomancer`、CAD関節名は`right_joint_1..7`・`left_joint_1..7`。

実機は既存のSpinal rosserial bridgeへ接続する。bridgeも起動する場合だけ、
`backend:=hardware connect_fc:=true fc_serial_port:=/dev/ttyUSB0`を指定する。
ファームウェアの書込みは行わない。校正・事前設定・停止・復旧は[実機手順](hardware.md)を参照。

実機の`joint_states`は14関節の表示用で、未対応関節はCAD q=0を含む。
形状操作には`measured_joint_states`を使う。校正済みかつ新鮮な実測関節だけを元の計測時刻で配信する。
viewer・Gazeboも同じ`measured_joint_states`を配信し、teleoperationと実験ログは全backendでこの入力を使う。
Gazeboは元の計測時刻を保持し、viewerは現在の仮想関節状態を配信する。
形状用の実測体幹点・固定軸合わせは別途設定が必要で、既定設定のままでは形状指令を開始しない。

## シミュレーション

[形状操作手順](shape.md)の`shape_demo.launch`はMk-II viewerとDRAGON Gazeboを起動する地上デモ。
`bringup.launch backend:=gazebo`はMk-II自身の物理シミュレーションで、両者を同じmasterへ重複起動しない。
`contact.launch`はMk-II viewer・DRAGON Gazebo・固定壁の接触観測用。実機へ接続せず、接触力覚は出力しない。
shape用のJacobianと直結力覚への接続は未実装。

## 検証

```bash
python3 -B robots/dracomancer/test/mkii/model.py
python3 -B robots/dracomancer/test/mkii/current.py
python3 -B robots/dracomancer/test/test_runtime.py
python3 -B robots/dracomancer/test/test_output.py
python3 -B robots/dracomancer/tests/test_shape.py -v
# ROS・Qt・Gazeboが利用できる隔離masterでのみ実行
bash robots/dracomancer/test/mkii/run.sh viewer
bash robots/dracomancer/test/mkii/run.sh gazebo
bash robots/dracomancer/test/mkii/run.sh direct
```

CADの15リンク・14関節とSTLの保存を検証する。モデル内のcontinuous関節、q=0、PID、collision、
2 Nm・2 rad/sはシミュレーション用の仮値であり、実機の保証値ではない。
自己衝突は無効。今回の検証結果・未確認事項は[統合記録](integration.md)を参照する。
