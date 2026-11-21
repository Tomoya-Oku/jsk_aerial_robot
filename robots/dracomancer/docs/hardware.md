# 実機の校正とトルク操作

標準経路、関節名、物理ID、軸数制限は[デバイス設計](device.md#関節と通信の仕様)を参照する。
ここではSpinal UART/RS485直結ノードの導入・校正・操作・復旧を扱う。

## 接続条件

`catkin build spinal dracomancer`でROS側とMCU用rosserialヘッダーを生成する。
MCUはボード型式に合うプロジェクトで別途ビルドする。
対応API・ビルド方法は[Spinalの文書](../../../aerial_robot_nerve/spinal/docs/current.md)を参照する。
旧ファームウェアに直結サービスがなければ本ノードは接続できない。

実機ノードは既存のrosserial bridgeを使い、自身でシリアルポートを開いたりファームウェアを書き込んだりしない。
同じサーボを制御する他の位置・電流・設定ノードを停止してから使用する。
Gazeboとは別のROS masterを使い、`use_sim_time=true`の環境では起動しない。

## 校正ファイル

`config/mkii/hardware.yaml`を別ファイルへコピーし、実機で確認した対応と校正値を設定する。

| 設定 | 内容 |
|---|---|
| `servo_ids` | CAD関節名→物理DYNAMIXEL IDの既定表。IDの書換え処理ではない |
| `actuators` | このノードが担当する実機関節。1台の直結経路の軸数制限内で定義する |
| `index` | Spinalのscan index。物理IDから推定しない |
| `id` | 物理IDの任意の上書き。省略時は同じ関節の`servo_ids`を使用 |
| `zero_tick`、`direction`、`radians_per_tick` | CAD q=0、正方向、関節rad/countの実測対応 |
| `joint_nm_per_amp` | 校正済みの関節出力Nm/A |
| `max_current_amp`、`max_torque_nm` | 承認した関節別上限 |
| `calibrated` | 校正が完了したときだけ`true`にする |

既定は`calibrated: false`・空の`actuators`。ID表だけを設定しても駆動しない。
ストール値由来の係数、シミュレーションのeffort/velocity、他関節の符号を校正値として流用しない。

## 起動と表示

### 未校正の動作確認だけを行う場合

既存のSpinal bridgeで`servo/states`と`get_board_info`を利用できれば、
電流制御APIやトルク校正なしで読み取り専用の仮表示を起動できる。

```bash
roslaunch dracomancer observe.launch
```

既定はルート名前空間のbridgeからID 1〜7を右腕の第1〜第7関節へ対応させる。
`get_board_info`で物理IDとscan indexを照合し、起動後最初の新鮮な全7軸計測を0度とする。
換算は直接駆動の4096 count/rev・全軸符号+1を仮定する。外部エンコーダ・異なる分解能比は拒否する。
実機のCAD零点・回転方向を校正した表示ではなく、反対側の腕はモデルの0度で表示する。
`arm:=left`、`servo_ids:="[10, 11, 12, 13, 14, 15, 16]"`、`spinal_ns:=/dracomancer`で接続先を変更できる。
表示先は`/dracomancer_preview`で、未校正データを`measured_joint_states`へ配信しない。
サーボへの指令、設定変更、Torque ON/OFF、ファームウェア書き込みは行わない。
サーボのエラー値は`preview/status`と警告ログへ表示する。計測が途切れた場合は表示を保持し、
新しい計測時刻を捏造しない。再起動するとその時点の姿勢を0度として取り直す。

### 校正済みの実機表示とトルク操作

```bash
source devel/setup.bash
roslaunch dracomancer bringup.launch backend:=hardware config:=/absolute/path/to/hardware.yaml
```

既存bridgeの接続先は`spinal_ns`、表示・操作側は`robot_ns`。標準はともに`dracomancer`。
bridgeを同時起動する場合の引数は[基本操作](usage.md#ビルドと基本操作)を参照する。

現行の実機ノードは1つのSpinal直結経路を扱い、最大10軸・scan index 0〜9を受け付ける。
左右7軸ずつの独立UART/RS485構成には、MCUの2系統化とホストの経路分離・計測統合が別途必要。
配線の分割や本ノードの二重起動だけでは両腕構成にならない。

実機のAnglesタブは読取り専用。未設定の関節は表示上のCAD q=0で、計測値にはしない。
`joint_states`は全体表示用、形態操作は`measured_joint_states`を使う。
後者は校正済み・新鮮・正常な計測関節だけを、最も古い有効関節計測時刻で配信する。
キャッシュ再配信で計測時刻を更新しない。

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
