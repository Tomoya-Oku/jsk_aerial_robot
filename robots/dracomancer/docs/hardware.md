# 実機の校正とトルク操作

標準経路、関節名、物理ID、軸数制限は[デバイス設計](device.md#関節と通信の仕様)を参照する。
ここではSpinal UART/RS485直結ノードの導入・校正・操作・復旧を扱う。

## 接続条件

`catkin build spinal dracomancer`でROS側とMCU用rosserialヘッダーを生成する。
MCUはボード型式に合うプロジェクトで別途ビルドする。
対応API・ビルド方法は[Spinalの文書](../../../aerial_robot_nerve/spinal/docs/current.md)を参照する。
Mk-II用H7 V2では既定の`DRACOMANCER_DUAL=1`でビルドする。
USART2/USART6とも1 Mbps、8N1、Protocol 2.0。バスごとのRS485変換回路が送受信方向を切り替える。
USART6のPC6/PC7が実基板で引き出せることと、既存回路・外付け回路の方向切替を確認する。
CubeIDEビルドには最新の生成済み`ros_lib`を使う。

実機ノードは既存のrosserial bridgeを使い、自身でシリアルポートを開いたりファームウェアを書き込んだりしない。
同じサーボを制御する他の位置・電流・設定ノードを停止してから使用する。
Gazeboとは別のROS masterを使い、`use_sim_time=true`の環境では起動しない。

## 校正ファイル

`config/mkii/hardware.yaml`を別ファイルへコピーし、実機で確認した対応と校正値を設定する。

| 設定 | 内容 |
|---|---|
| `servo_ids` | CAD関節名→物理DYNAMIXEL IDの既定表。IDの書換え処理ではない |
| `actuators` | このノードが担当する実機関節。両腕構成ではCAD関節名の先頭の`right_`・`left_`でバスを選ぶ |
| `index` | 各バスのscan index。起動時に`get_board_info`で照合する |
| `id` | 物理ID。省略時は同じ関節の`servo_ids`を使用。両腕構成では`servo_ids`との一致が必要 |
| `zero_tick`、`direction`、`radians_per_tick` | CAD q=0、正方向、関節rad/countの実測対応 |
| `joint_nm_per_amp` | 校正済みの関節出力Nm/A |
| `max_current_amp`、`max_torque_nm` | 承認した関節別上限 |
| `calibrated` | 校正が完了したときだけ`true`にする |

既定は`calibrated: false`・空の`actuators`。ID表だけを設定しても駆動しない。
ストール値由来の係数、シミュレーションのeffort/velocity、他関節の符号を校正値として流用しない。

## 起動と表示

Spinalを単独で接続する場合は、シリアルポートを指定してDracomancer用bridgeを起動する。

```bash
roslaunch dracomancer bridge.launch serial_port:=/dev/ttyUSB0
```

bridgeは左右のサーボとJoystickを`/dracomancer/right/`、`/dracomancer/left/`配下に登録する。
命名はROS側で解決するため、既存のファームウェアを使える。

### 未校正の動作確認だけを行う場合

各腕の計測経路を読み取り、装着前に関節とIDの対応を確認できる。

```bash
roslaunch dracomancer observe.launch arm:=right
roslaunch dracomancer observe.launch arm:=left
```

既定は`/dracomancer`のbridgeからID 1〜7を右腕の第1〜第7関節へ対応させる。
`get_board_info`で物理IDとscan indexを照合し、起動後最初の新鮮な全7軸計測を0度とする。
換算は直接駆動の4096 count/rev・全軸符号+1を仮定する。外部エンコーダ・異なる分解能比は拒否する。
実機のCAD零点・回転方向を校正した表示ではなく、反対側の腕はモデルの0度で表示する。
`arm:=left`の既定IDは11〜17。bridgeを別の名前空間で起動した場合は`spinal_ns`を指定する。
表示先は`/dracomancer_preview`で、未校正データを`measured_joint_states`へ配信しない。
サーボへの指令、設定変更、Torque ON/OFF、ファームウェア書き込みは行わない。
サーボのエラー値は`preview/status`と警告ログへ表示する。計測が途切れた場合は表示を保持し、
新しい計測時刻を捏造しない。再起動するとその時点の姿勢を0度として取り直す。

### 校正済みの実機表示とトルク操作

```bash
source devel/setup.bash
roslaunch dracomancer bringup.launch backend:=hardware config:=/absolute/path/to/hardware.yaml
```

既定のbridgeは`/dracomancer/right/servo/states`と`/dracomancer/left/servo/states`を配信する。
`spinal_ns`でbridgeの名前空間、`robot_ns`で表示・操作の名前空間を指定できる。
bridgeの同時起動は`connect_fc:=true`と`fc_serial_port`を指定する。
従来の1バスファームウェアを使用する場合は`dual_spinal:=false`と、その機体の物理IDを設定する。

左右バスの計測を関節名で集約し、両腕のサンプルがある場合は時刻差50 ms以内の計測を採用する。
この閾値はホストの初期設定で、実測の通信周期に基づく評価は未実施。

実機のAnglesタブは読取り専用。未設定の関節は表示上のCAD q=0で、計測値にはしない。
`joint_states`は全体表示用、形態操作は`measured_joint_states`を使う。
後者は校正済み・新鮮・正常な計測関節だけを、最も古い有効関節計測時刻で配信する。
キャッシュ再配信で計測時刻を更新しない。

### Joystickの左右校正

右は`/dracomancer/right/joystick/raw`、左は`/dracomancer/left/joystick/raw`。
Spinalの起動時は両方を中立にする。
左は先にターミナルで校正し、左専用のファイルに保存する。

```bash
rosrun dracomancer calibrate_joystick.py _js_raw_topic:=/dracomancer/left/joystick/raw \
  _js_calibrated_topic:=/dracomancer/left/joystick/calibrated \
  _yaml_path:="$HOME/.ros/dracomancer-joystick-left.yaml"
```

保存後に校正ノードを終了し、両腕の操作を使う場合は`teleop_mode:=true`、
`mapping_mode:=dual_joint`または`dual_shape`、`enable_position_control:=true`を指定する。

## トルク操作と停止

1. Nmを入力して関節のチェックをONにする。機種・ID・モード・電流上限を確認し、モード0を準備する。
   準備だけではTorque ONにしない。
2. HOLDを押すとARMし、押下中の新しい指令だけを生電流へ変換する。
   読み戻したCurrent LimitもNm入力上限へ反映する。
3. 解放、ZERO、ウィンドウ終了でゼロ電流／Torque OFFを要求する。

Nmは要求値で、トルクセンサの実測値ではない。
ホスト150 ms、Spinalは独立したHAL時計で250 msの指令期限を監視する。
期限切れ後のworker周期でゼロ／OFFを送り、失敗時は再試行する。
実際の停止時間はRTOS周期・UART遅延を含み、実機で測定する必要がある。

古いセッション番号・sequenceは再駆動に使えず、PREPARE/ARMをやり直す。
電流制御の所有中は旧位置・Torque ON・設定処理の介入を防ぐ。
同一PC・同一ユーザー・同じ一時ディレクトリとROS_MASTER_URIのノードは、電流topicとモードserviceを排他取得する。
後発ノードは出力・終了処理の登録前に停止し、動作中の所有者へゼロ／OFFを送らない。
ファイルロックによる複数PC間の排他は実装していない。

## 元の位置モードへの復旧

チェックOFFはTorque OFFにし、モード0を保持する。EEPROMを暗黙には戻さない。
他の位置指令ノードを停止した状態で、次のサービスを明示的に呼ぶ。

```bash
rosservice call /dracomancer/control/right_joint_1/restore_position
```

PREPARE時に保存できた元の位置モード3/4、PID、Profileを復旧し、Torque OFFで返す。
起動時点ですでにモード0だった場合、元の設定が分からなければ復旧を拒否する。
復旧後は別の位置指令で再びONになり得るため、先に他の制御ノードを停止する。

導入前にボード型式、元ファームウェア、モード・PID・Profile・Current Limitを保存する。
ソフトウェア監視とは別に、物理的な電源遮断・離脱手段を用意する。
接触力覚の自動出力の対応範囲は[ソフトウェア設計](software.md#接触安全力覚の設計)を参照する。
