# Spinal UART直結のトルク操作

対象はSpinal → UART → RS485 → XM430-W350-R。
研究仕様は [4章](../../dracomancer/docs/design-spec/04-hardware.md) と
[5章](../../dracomancer/docs/design-spec/05-software.md) を参照。
この手順はソフトウェアの導入を扱い、実機へのファームウェア書込みを実行しない。

## 対応範囲

- `aerial_robot_nerve/spinal` のDirectServoModeサービスとDirectServoCurrentメッセージを使う。
- 新しい直結APIはSTM32H7 / H7 V2の構成でリンク確認済み。
- 既存F7構成はビルド可能だが、mainにDirectServoの起動処理がなく直結対応には含めない。
- ボード型式、UART/RS485配線と応答時間、モーター対応と校正値、物理的な駆動・停止は未確認。
- 1台のDirectServoが扱う上限は現在10個。CADの14関節を1台へ割り当てる設定は拒否する。

`catkin build spinal dracomancer_mkii_ros` でROS側とMCU用rosserialヘッダーを生成する。
MCUは型式に合うCubeIDEプロジェクトで別途ビルドする。
ビルド方法と通信APIは [Spinalの文書](../../../aerial_robot_nerve/spinal/docs/current.md) を参照。
旧ファームウェアには新サービスがなく、実機用ノードは接続できない。

## 校正と起動

旧Mk-Iの位置指令／力覚ノードを停止し、Spinalのrosserial bridgeだけを起動して使う。
本パッケージはシリアルポートを開かず、既存ROSサービスへ接続する。
Gazeboとは別のROS masterを使用する。`use_sim_time=true` の環境では実機ノードを拒否する。

`config/hardware.yaml` を別のファイルへコピーし、実機で確認した次の値を設定する。

- URDF関節とSpinal scan index／DYNAMIXEL bus IDの対応（indexとIDは別物）
- CADのq=0に対応するエンコーダ値、正方向、関節rad/count
- 校正済みの関節出力Nm/A、使用する最大電流と最大トルク

既定は `calibrated: false`、空の対応表で駆動しない。
ストール値から推定したNm/Aを校正値としては扱わない。

```bash
source devel/setup.bash
roslaunch dracomancer_mkii_ros hardware.launch config:=/absolute/path/to/hardware.yaml
# 既存パッケージの入口を使う場合
roslaunch dracomancer mkii.launch backend:=hardware config:=/absolute/path/to/hardware.yaml
```

`spinal_ns:=/dracomancer` が接続先、`robot_ns:=dracomancer_mkii` が表示・操作側。
未設定の関節はCADのq=0で表示し、計測値としては扱わない。実機のAnglesタブは読取り専用。

## 子ウィンドウ

1. Nmを入力して関節のチェックをONにする。機種・ID・モード・電流上限を確認し、モード0を準備する。
   準備だけではTorque ONにしない。
2. HOLDボタンを押すとARMし、押下中の新しい指令だけを生電流へ変換する。
   読み戻したCurrent LimitもNm入力上限へ反映する。
3. 解放、ZERO、ウィンドウ終了時はゼロ／OFFを要求する。

指令値の単位はNmだが、トルクセンサの実測値ではない。
ホストは150ms、Spinalは独立したHAL時計で250msの指令期限を監視する。
Spinalは期限切れ後のworker周期でゼロ／OFFを送り、失敗時は再試行する。
実際の停止時刻はRTOS周期・UART遅延を含み、実機で未測定。

古いセッション番号・sequenceは再駆動に使えず、再度PREPARE/ARMが必要。
電流制御の所有中は旧位置・トルクON・設定処理の介入を防ぐ。
ソフトウェア監視は認証済み非常停止ではなく、物理的な電源遮断手段が別途必要。

## 復旧

チェックOFFはTorque OFFとしてモード0を保持し、EEPROMを暗黙に戻さない。
旧指令ノードを停止した状態で、明示的な
`/dracomancer_mkii/control/<joint>/restore_position`（std_srvs/Trigger）を呼ぶと、
PREPARE時に保存できた元の位置モード3/4、PIDとProfileを復旧し、Torque OFFで返す。
起動時点ですでにモード0だった場合、元の設定が分からず復旧を拒否する。
復旧後は旧位置指令で再びONになり得るため、制御ノードの停止を先に確認する。

実機導入前にボード型式、元ファームウェア、サーボのモード・PID・Profile・Current Limitを保存する。
今回の作業では書込み・駆動を行っていない。
