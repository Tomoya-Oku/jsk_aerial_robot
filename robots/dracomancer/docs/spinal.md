# Mk-IIのSpinal構成調査

## 実装

H7 V2のMk-II構成では、USART2の右7軸とUSART6の左7軸を独立したRS485ドライバで扱う。
ファームウェアはCANの名前との衝突を避けるため、RS485の各エンドポイントに `right/`・`left/` を付ける。Dracomancer用bridgeが下表の公開名に変換する。
単一バス版の `servo/states` を使う場合は `dual_spinal:=false` を指定する。

| 用途 | 右 | 左 |
|---|---|---|
| サーボと物理ID | USART2、1〜7 | USART6、11〜17 |
| 状態 | `/dracomancer/servo/states/right` | `/dracomancer/servo/states/left` |
| トルク状態 | `/dracomancer/servo/torque_states/right` | `/dracomancer/servo/torque_states/left` |
| 電流 | `/dracomancer/servo/direct_current/right` | `/dracomancer/servo/direct_current/left` |
| モード | `/dracomancer/direct_servo_mode/right` | `/dracomancer/direct_servo_mode/left` |
| 接続情報 | `/dracomancer/get_board_info/right` | `/dracomancer/get_board_info/left` |
| Joystick | I2C1、`/dracomancer/joystick/raw/right` | I2C3、`/dracomancer/joystick/raw/left` |
| Joystickの±1変換 | `/dracomancer/joystick/scaled/right` | `/dracomancer/joystick/scaled/left` |
| Joystick校正出力 | `/dracomancer/right/joystick/calibrated` | `/dracomancer/left/joystick/calibrated` |

Joystickの校正済み押下は各`calibrated/pressed`に配信する。bridgeはファームウェアの`joystick/raw`と
`joystick/left/raw`を左右別のパスにリマップする。

U024-V2の`raw`は`[Xの差分カウント, Yの差分カウント, ボタンレベル]`。
Spinalは起動時の中心をX・Yから引き、レジスタ`0x20`のボタンを未押下`1`・押下`0`で送る。
Dracomancer用bridgeは各`raw`を`scaled`（`sensor_msgs/Joy`）に変換する。
`axes[0:2]`はX・Yで、`int16`の範囲を`-1`〜`1`へ線形換算する。
`buttons[0]`は押下`1`・未押下`0`。タイムスタンプはホストの受信時刻。
実測した可動範囲やデッドゾーンを適用した値は`calibrated`を使う。
[M5Stack公式実装](https://github.com/m5stack/esphome-yaml/blob/main/components/i2c_joystick_2/i2c_joystick_2.cpp)

各ドライバは専用のDMAバッファと通信タスクを持つ。起動時に各バスの7台と物理IDを照合し、scan indexを0〜6に割り当てる。
既存メッセージの時刻は位置読取の一巡を始めた時点を表す。応答した軸を配信し、欠損した軸の古い値は再配信しない。
ホストは左右でindexが同じ場合もバスで識別し、起動時に実際の物理IDと校正表を照合する。

Mk-IIのドライバは既存のSpinal Flash配置を使わず、サーボ本体のレジスタを使用する。
接続情報と現行の電流セッションAPIを公開する。旧位置指令と`direct_servo_config`は`DRACOMANCER_DUAL=0`の構成で使用する。
UART速度とモードはMk-IIの初期化時に変更するため、CubeMXのUSART6基本設定は旧構成と共通にできる。
RS485変換器の方向制御は回路側で行う前提。USART6の実基板上の引き出し、VIM4との実接続、通信周期と停止遅延は未確認。
起動と校正は[hardware.md](hardware.md)を参照。

## 実装前の調査記録

調査日: 2026-11-26。対象基板は指示者確認済みのSTM32H7 V2。
以下は変更前のローカルソースの静的調査と当時の提案。
既存設計の入口は[device.md](device.md)、[software.md](software.md)。
現在の実装との差は上の「実装」を参照。

## 結論

指定構成にはSpinalファームウェアの変更が必要。
現行はUSART2のDirectServo 1個、I2C1のJoystick 1個だけを初期化する。
左右7軸を独立バスに分ければ、1ドライバの`MAX_SERVO_NUM=10`は変更不要。
ただしドライバを2個作るだけでは、共有DMA受信バッファと固定ROS名が衝突する。
Khadas VIM4側にも、左右の通信経路・校正・計測統合の追加が必要。

## 指定構成と現行との差

| 用途 | 指定ポート | 現行実装・必要な対応 |
|---|---|---|
| 右腕XM430×7 | USART2 | 1 Mbps、TX/RX、DirectServo実装あり |
| 左腕XM430×7 | USART6 | 115200 bps、RXのみ。サーボ用ドライバ・ROS経路を追加 |
| 右Joystick | I2C1 | U024-V2用クラスと初期化あり |
| 左Joystick | I2C3 | 周辺回路初期化あり。Joystickインスタンス・ROS経路を追加 |

ソース上のピンはUSART2がPD5/PD6、USART6がPC6/PC7、
I2C1がPB8(SCL)/PB9(SDA)、I2C3がPA8(SCL)/PC9(SDA)。
USART2 RXはDMA1 Stream4、USART6 RXはStream3で、どちらも循環DMAが既に設定されている。
USART6のTXピンもMSPでは設定済みなので、GPIOを一から追加する必要はない。

### IDの解釈

Wizardで設定する物理IDと、Spinalの配列番号であるscan indexを分ける。
受信パケットのIDはそのまま`ServoData::id_`に保存され、`get_board_info`の`ServoInfo.id`へ返される。
`servo/states`の`ServoState.index`は検出配列の番号で、0始まり。
ファームウェアに「物理IDから一律1を引く」処理はない。

| 腕 | Wizard・通信パケット・ServoInfo.id | 各バスのscan index |
|---|---|---|
| 右 | 1〜7 | 7台検出なら0〜6。各関節との対応は実測 |
| 左 | 11〜17 | 7台検出なら0〜6。各関節との対応は実測 |

`index=0, servo_id=11`は左バスの先頭が物理ID11だった場合の有効な組合せ。
左をindex 10〜16として扱わない。欠損・検出順に依存するため、`index=id-1`も使用しない。
配線順・上からの関節順は物理IDとCAD関節名の対応表で指定する。

調査時点の設定では、物理IDが右0〜6・左10〜16だった。
現行の`config/device/hardware.yaml`と`device.md`では右1〜7・左11〜17にそろえている。

## 実装提案

### 1. 基板設定とRS485回路

- Mk-IIの2バス構成をビルド設定で選択し、既存1バス構成を維持する。
- `spinal.ioc`と生成された`main.c`を整合させ、USART6を1 Mbps・8N1・TX/RXにする。
  現ドライバも初期化時にUART設定を書き換えるが、CubeMX設定にも用途を反映する。
- USART6はDShot ESC telemetryと共用できない。現設定は`DSHOT=0`。
  左サーボ有効時とtelemetry有効時をビルド時に排他にし、将来の設定変更で重複させない。
- 各UARTに独立したRS485変換回路が必要。RS485のDATA+/DATA-へMCUのTX/RXを直結しない。
  外付け回路が自動方向制御ならその仕様に従い、DE/REが必要ならバスごとのピンを渡して、
  最終ビットの送信完了後に受信へ切り替える。
- `pinReconfig()`は未検出時にTTLへ切り替え、H7 V2ではPD5を直接再設定する。
  USART6を渡しても右UARTのピンを変更してしまう。Mk-IIでは明示したRS485モードに固定し、
  未接続時にTTL探索へ入らない経路を追加する。従来の自動探索は既存構成用に残す。

同梱`boards/stm32H7_v2/spinal.pdf`は2021年版回路図。
I2C1/I2C3のコネクタは確認できるが、PC6/PC7の外部接続や現物のRS485回路は確定できない。
実物と一致する改版の回路図・配線で確認する。USART2側も既存の外付け回路を確認対象とする。
XM430のRS485回路要件は[ROBOTIS資料](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#communication-circuit)を参照。

### 2. DYNAMIXELドライバを2インスタンス対応にする

- `dynamixel_serial.cpp`のファイルスコープ`rx_buf_[512]`をバスごとに分離する。
  案は32バイト境界に配置した`rx_pool[2][512]`を既存`.ServoRxBufferSection`に置き、
  各ドライバへ別のポインタを渡す方式。通常ヒープのメンバ配列へ移すだけではDMAとcacheの条件を満たすとは限らない。
- 現リンカ設定では同sectionの開始が`0x24044800`、次sectionが`0x24044C00`。
  512×2はこの1 KiB内に収まる案で、既存MPU設定も当該1 KiBを非cache領域にしている。
  実ビルドのmapで境界・配置・重複を確認し、必要な場合にリンカ/MPU設定を変更する。
- 読取位置、UART、命令queue、受信解析状態、電流セッション、排他制御は左右で独立させる。
  既にメンバになっている状態はその構造を維持し、受信再開処理も各バスのバッファを使う。
- 検出結果を指定ID集合と照合する。7台未満・余分なID・重複では関節を詰め替えず、
  対応未確定として駆動を許可しない。`MAX_SERVO_NUM`は10のまま。

### 3. DirectServoのROS経路を分離する

`DirectServo`へ省略可能なprefixを追加し、既存の右側APIを保持する案を推奨する。
以下はNodeHandleから見た相対名。最終的な絶対名はbridgeの名前空間とremapに合わせる。

| API | 右・既存名 | 左・追加名の案 |
|---|---|---|
| 計測 | `servo/states` | `left/servo/states` |
| 電流指令 | `servo/direct_current` | `left/servo/direct_current` |
| モード操作 | `direct_servo_mode` | `left/direct_servo_mode` |
| 接続情報 | `get_board_info` | `left/get_board_info` |
| 設定 | `direct_servo_config` | `left/direct_servo_config` |

`servo/target_states`、`servo/torque_enable`、`servo/torque_states`、`joint_profiles`も分離する。
生成した名前の文字列はインスタンスの寿命中保持する。
既存msg/srvはバス別エンドポイントで使用でき、bus番号を追加するためのMD5変更は不要。
単一の0〜13 indexへ統合する案は、既存ホスト・API・上限検査の変更が増えるため推奨しない。

### 4. 通信タスクと設定保存

- `main.c`に左右のDirectServoを置く。初期化と定常運転の両方でバスを分離する。
  片側未接続で全サーボタスクを終了する現条件も変更する。
- 左右を単一ループで順番に呼ぶだけでは、片側の応答待ちが反対側の更新を遅らせる。
  バスごとのworkerと有限時間の通信処理を用意し、待機時にはRTOSへ処理を返す。
  既存のbusy waitを残したままタスクを2個にしても独立性は保証されない。
- 現H7 V2は`servo_.init(..., NULL)`で、モード要求がcallback側でも直接I/Oする。
  RTOS開始前の探索と、開始後のworker専有I/Oを分け、実在するバス別mutex/要求queueを設定する。
  ROS callbackは要求を渡すだけにし、電流送信・モード変更・停止を同一バス内で直列化する。
- 現行250 msの電流指令期限、session/sequence検査、起動時の自動ARM抑止を左右で維持する。
  実際の停止遅延にはworkerの待ち時間も含まれるため、通信断試験で計測する。
- 共通NodeHandleへのpublishは既存core処理も含めて直列化を確認する。
  バスworkerからは計測スナップショットを渡し、共通の発行処理でserialize/publishする構造を候補とする。
  TX DMAを追加する場合は`STM32Hardware.cpp`の全UART共通callbackがrosserial用semaphoreを
  無条件に解放する点も修正する。最初の段階ではサーボTXにDMAを追加する必要はない。
- `FlashMemory`は`addValue()`の登録順でデータを連続保存する。
  左ドライバ追加で後続データの位置が変わり、左の新領域も既存Flashから無条件に読み込まれる。
  既存IMU・右サーボ設定の位置を維持した固定の左右ブロック、形式version、妥当性検査と移行を設計する。
  旧形式・未初期化領域を検出したら駆動を許可しない。Flash全消去で既存校正を暗黙に失わせない。
- 飛行制御器はDirectServoを1個だけ参照する。Mk-II用サーボを飛行側のgimbal指令で駆動しないよう、
  `controller_.init()`への接続と所有者を確認する。両腕操作のために飛行制御APIを14軸へ広げる必要はない。

### 5. Joystickを2系統にする

- `I2CJoystick`へtopic名を渡せるようにし、I2C1とI2C3で各1個初期化する。
  既存`joystick/raw`を右として維持し、左は`joystick/left/raw`を追加する案。
  各インスタンスが持つ中心値・メッセージ配列はそのまま分離して使える。
- U024-V2の既定7 bitアドレスは`0x63`。独立I2Cなので両方同じアドレスでよく、
  アドレス変更・I2C multiplexerの追加は不要。
  [M5Stack公式仕様](https://docs.m5stack.com/en/unit/Unit_Joystick2)を参照。
- I2C1は気圧計、I2C3はIMUの外付け磁気センサ探索/読取でも使用する。
  Joystick専用として既存センサを無断で切り離さず、同一バスへのアクセスを直列化する。
  既存I2C Fast設定とJoystick、pull-up、電源・信号電圧の整合も実機で確認する。
- 現在は1 kHz coreTaskから100 Hzで同期読取し、1回のHAL読取timeoutが3 ms。
  2台の同期読取をそのまま増やすとcore周期を超え得る。
  小さな非同期状態機械などでcoreTaskの応答待ちを除き、既存センサとのbus所有権を管理する。
  別タスクへ移す場合も、既存センサ側を含めたmutex/queue対応が必要。
- 起動校正の`ok_cnt`は現在boolで、一部読取失敗時にも総和を64で割ってしまう。
  成功数で平均し、必要サンプル不足時は校正未成立として発行/操作を抑止する。
  読取失敗で古い入力を再発行せず、ホストが通信断を検知できるようにする。

### 6. VIM4上のROS設定・統合

- 既存`spinal/bridge.launch`のrosserial経路を利用する案。USART1は既存ホスト通信用として保持する。
  VIM4との物理接続方式、OS/ROS環境、実デバイス名、921600 bpsでの動作は別途確認する。
- `direct.py`は1つのRPC・電流topic・計測topicと、全関節で一意のindexを前提としている。
  2バスを1インスタンスへ渡すなら、設定を`(bus, index, physical_id)`に拡張し、
  busごとにRPC・publisher・subscriber・排他所有権を分離する。
  同じindex=0が左右にあっても衝突させず、同じ名前の関節を2経路から更新しない。
- 校正済みの左右7関節をCAD関節名で統合する。各バスの元計測時刻と受信時刻を保持し、
  左右の時刻差・鮮度を検査して、必要な14関節が揃った場合だけ両腕の形態入力を成立させる。
  未計測側を0で補って`measured_joint_states`へ出さない。
- `joystick.py --calibrate`を左右別の入力・校正ファイルで起動する。
  右の既存`/dracomancer/joystick/calibrated`と、左の
  `/dracomancer/joystick/left/calibrated`へ接続する。後者の消費側は既に実装がある。

## 実装順と確認項目

1. 配線・RS485回路・実機物理IDを確認し、未接続状態も含むバス分離とFlash形式を設計する。
2. USART6、バッファ、ドライバ、ROS名を追加し、トルクOFFで左右の接続情報と計測を確認する。
3. バスworkerと電流セッションを分離し、片側断線・片側未接続・古い指令・再起動を試験する。
4. Joystickを2個にし、中心校正、一部読取失敗、押下、中立、片側I2C障害を確認する。
5. VIM4で左右計測・Joystickを統合し、時刻差・周期・帯域・通信断を測る。
6. 校正・電流上限が揃ってから、片軸、片腕、両腕の順に力覚を確認する。

ソフトウェア試験は左右バッファ/ID/indexの分離、誤バス指令拒否、Flash移行、
Joystickの部分失敗時の校正、古い計測の統合拒否を対象とする。
従来1バス構成のビルド・既存`spinal_current`試験も維持する。
実機では片側障害が反対側の更新・停止を遅らせないかを重点確認する。
停止を片腕単位/両腕一括のどちらで連動させるかと、許容時刻差・周期の数値は実装前に指示者と確定する。

## 主な変更対象

| ファイル | 変更理由 |
|---|---|
| `boards/stm32H7_v2/spinal.ioc`、`Src/main.c` | USART6設定、左右初期化、task、Joystick経路 |
| `lib/Jsk_Lib/configs/STM32H7_v2/config.h` | Mk-II構成とtelemetryの排他 |
| `lib/Jsk_Lib/servo/drivers/Dynamixel/dynamixel_serial.{h,cpp}` | バッファ、RS485初期化、worker、設定登録 |
| `lib/Jsk_Lib/servo/servo.{h,cpp}` | ROS名、計測スナップショット、左右の独立制御 |
| `lib/Jsk_Lib/flashmemory/flashmemory.{h,cpp}` | 保存形式と移行。変更範囲は設計後に確定 |
| `lib/Jsk_Lib/sensors/joystick/joystick.h` | topic、校正、非同期読取 |
| I2C共有センサの実装 | 別task/非同期化する場合のbus直列化 |
| `robots/dracomancer/scripts/device/direct.py`、設定・launch | bus識別、物理ID、左右計測・Joystick統合 |
| `device.md`、`software.md`、`hardware.md`、`roadmap.md` | 最新仕様と実装・未検証範囲の整合 |

`boards/`・`lib/`は`aerial_robot_nerve/spinal/mcu_project/`からの相対パス。
リンカ/MPU/MSP/割込み設定の変更要否は、確定したRS485回路と実ビルドで判断する。
実装はSpinalとDracomancerを跨ぐため、着手時には上位AGENTS.mdに従って範囲を確認する。

## 調査範囲・未確認事項

上記ファイル、既存ホスト設定、同梱回路図のテキスト、メーカー公式資料を確認した。
基板現物・配線・サーボとの通信・VIM4環境・MCUの実ビルド/書き込み・停止遅延は未確認。
`GIT.md`はローカル探索で見つからず、Gitコマンドは実行していない。調査対象は現在の作業ツリー。
本変更は調査文書の新規追加だけで、Catkin再ビルドは不要。
実装時はMCUをSTM32CubeIDE等で別途ビルド・書き込みする必要がある。
msg/srvを維持する案では型の再生成は不要だが、ホストの変更に応じて関連Catkinパッケージを検証する。
