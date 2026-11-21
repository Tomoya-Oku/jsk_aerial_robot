# デバイス設計

## 構成

主デバイスは座位で使用する右腕7 DoFの上肢外骨格型Dracomancer Mk-II。
人体関節状態・腕の相対形状を取得し、校正済みの関節にはトルクを提示する。
CADには両腕の15リンク・14関節があり、新STL 8種類を`meshes/`から使用する。
標準入力は`right_joint_1..7`。両腕方式では左腕も含め、入力ベクトルを左7軸・右7軸の順に並べる。

モデルの正本は`urdf/cad.urdf`、world固定・transmission・Gazebo設定を加えたモデルは`urdf/mkii.urdf.xacro`。
CADの慣性・軸・関節変換とSTLハッシュは`config/mkii/cad.json`と`test/mkii/model.py`で確認する。
continuous関節、CAD q=0、URDFのeffort・velocity、PID、減衰、箱collisionはシミュレーション用の仮設定。
実機の可動限界・許容トルクを保証せず、自己衝突判定も有効化していない。

## 関節と通信の仕様

| 項目 | 仕様 |
|---|---|
| 能動関節 | 右腕7軸、各軸ROBOTIS XM430-W350-R |
| 右腕の物理ID | `right_joint_1..7`を順に0〜6 |
| 左腕の物理ID | `left_joint_1..7`を順に10〜16 |
| 位置分解能 | 4096 pulse/rev [10] |
| トルク提示モード | Current Control Mode、Operating Mode = 0 [10] |
| 電流指令単位 | 2.69 mA [10] |
| 標準経路 | Spinal → UART/RS485 → XM430 |
| 接続API | `direct_servo_mode`、`servo/direct_current`、`servo/states` |
| 標準名前空間 | `/dracomancer` |

物理IDの既定値は`config/mkii/hardware.yaml`の`servo_ids`が管理する。
Spinal scan indexは各通信系統の接続時に確認する別番号で、物理IDから推定しない。
DirectServoは1系統最大10軸（index 0〜9）。1系統14軸への拡張は取り消した。
両腕の実機構成は、Spinal 1台から独立したUART/RS485を2系統用意し、左右7軸ずつ接続する方針。
各系統のscan indexは実測して設定する。同一バスの配線分岐は独立した2系統にはならない。
現行MCUのDirectServo初期化は1系統だけで、2系統のドライバ・ROS経路・計測統合は未実装。
第2 UARTの割当とRS485回路、左右の時刻整合、通信周期・電源・停止動作の実機検証が必要。
直結APIの対象はSTM32H7/H7 V2。F7の直結実機動作は確認できていない。

12 Vのstall torque 4.1 Nm・stall current 2.3 Aから得る公称換算値は約1.783 Nm/A [10]。
実機の校正値・連続使用可能トルク・人体への安全値には転用しない。
方向、摩擦、実トルク、温度を関節ごとに測定し、ソフトウェア上限とサーボのCurrent Limitを定める。
設定と駆動条件は[実機手順](hardware.md)に従う。

## 機械設計の要件

以下は装着機構の設計・評価要件で、CAD表示やソフトウェア試験だけで達成済みとは扱わない。

- 上腕・前腕カフ、上腕・前腕リンク長調整、肩基準位置調整を用意する。
- 人体の軸回旋を拘束しないよう、カフと外骨格の間に受動回転自由度または回転スライド機構を設ける。
- 関節軸のずれ、装着面、身体寸法への適応を評価する [8]。
- 機械ストッパ、物理的な電源遮断、装着者が自力で離脱できる構造を確認する。

軽量化は末端慣性、関節摩擦、総質量の順に重視する。重量物を肩・体幹側に寄せ、
前腕・手先の構造、外装、配線を軽量化する。中空アルミ・CFRP等を候補とする。
評価には総質量に加え、肩関節回りの等価慣性を用いる。

身体寸法への調整には、telescopic link、長穴とクランプ、index plunger、quick releaseを候補とする。
調整範囲・カフ径・装着時間等は人体寸法と実測から決める。
未確定値と両腕化の条件は[残る開発・検証](roadmap.md)で管理する。

## 参考文献

[8] L. Chen, D. Zhou, and Y. Leng,
“A Systematic Review on Rigid Exoskeleton Robot Design for Wearing Comfort: Joint Self-Alignment, Attachment Interface, and Structure Customization,”
*IEEE Transactions on Neural Systems and Rehabilitation Engineering*, vol. 32, pp. 3815–3827, 2024.
DOI: 10.1109/TNSRE.2024.3479283.

[10] ROBOTIS, “XM430-W350,” *DYNAMIXEL e-Manual*.
https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
