# デバイス設計

## 構成

主デバイスは座位で使用する右腕7 DoFの上肢外骨格型Dracomancer Mk-II。
人体関節状態・腕の相対形状を取得し、校正済みの関節にはトルクを提示する。
2026年11月27日保存のSolidWorksアセンブリから取り出した両腕モデルは15リンク・14関節。
基部1点、左右共用の肩・上腕・前腕5点、左右別の手首4点のSTLを`meshes/`直下に置く。
標準入力は`right_joint_1..7`。両腕方式では左腕も含め、入力ベクトルを左7軸・右7軸の順に並べる。

CAD基準は`urdf/cad.urdf`。従来の`urdf/assembly.urdf`も同じファイルを指す。
`urdf/mkii.urdf.xacro`は同じメッシュ・関節フレーム・慣性を使い、world固定・transmission・Gazebo設定を加える。
CADの慣性・軸・関節変換とSTLハッシュは`test/mkii/model.py`で照合する。
Gazeboの衝突判定にはSTLの外接箱を使う。URDFのeffort・velocity、PID、減衰はシミュレーション用の仮設定。
実機の可動限界・許容トルクを保証せず、自己衝突判定も有効化していない。

## 関節と通信の仕様

| 項目 | 仕様 |
|---|---|
| 能動関節 | 左右各7軸、各軸ROBOTIS XM430-W350-R |
| 右腕の物理ID | `right_joint_1..7`を順に1〜7 |
| 左腕の物理ID | `left_joint_1..7`を順に11〜17 |
| 位置分解能 | 4096 pulse/rev [10] |
| トルク提示モード | Current Control Mode、Operating Mode = 0 [10] |
| 電流指令単位 | 2.69 mA [10] |
| 標準経路 | Spinal → UART/RS485 → XM430 |
| 接続API | `right/`・`left/`配下の`direct_servo_mode`、`servo/direct_current`、`servo/states` |
| ROS名前空間の既定 | Spinal bridgeとMk-IIの表示・操作は`/dracomancer` |

物理IDの既定値は`config/device/hardware.yaml`の`servo_ids`が管理する。
Spinal scan indexは各通信系統の接続時に確認する別番号で、物理IDから推定しない。
各DirectServoの上限は10軸。STM32H7 V2のMk-II構成は右USART2・左USART6の独立RS485を各7軸で使う。
起動時に各バスの所定の7 IDを確認し、物理ID順でindex 0〜6を割り当てる。
右JoystickはI2C1、左JoystickはI2C3に接続する。ROSホストはKhadas VIM4を想定する。
旧1バス構成の直結APIはSTM32H7/H7 V2用。F7での直結実機動作は未確認。
第2 UARTの端子・RS485の方向切替、左右の時刻差、通信周期・電源・停止動作は実機検証が必要。

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
