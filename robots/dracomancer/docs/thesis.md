# 修士論文との対応

制御理論の正本は`C:/Users/tomoy/papers/master_thesis`のTypst原稿。
機構・関節座標は`chapters/03-device.typ`、写像・力覚・判定・補償は`chapters/04-control.typ`を参照する。
PDFや旧設計資料を優先して原稿の理論を変更しない。
参照した第4章のSHA-256は`da93d58f898142aebb121bd6d0f9750284e3bef9f4ac23e51698bb1bbdcf3e72`。
作業中の改稿で追加された視点・傾き入力面と複数接触の定義も対象とする。
中心線と接触Jは固定FC向きの重心系で表し、予測モデルのC軸への変換はサービスの`fc_to_cog`で行う。
位置補償のworld向きは、実測baselink odometryとURDFの固定FC変換から求める。
予測モデルの仮想CoG軸と推定器のCoG軸を同一と仮定して二重回転させない。

## 実装対応

| 修論の式ラベル | 実装 |
|---|---|
| `legacy-wrist`、`legacy-elbow`、`legacy-shoulder` | `joint_mapping.py`の片腕関節角方式 |
| `normalized-centerline`、`retarget-objective` | `shape_mapping.py`の片腕中心線方式と有界最適化 |
| `bimanual-direct-map` | 左・右の順に14軸を受け、右肘／両肩差／左肘を割り当てる関節角方式 |
| `bimanual-centerline`、`bimanual-centered-centerline` | 左手→右手の中心線。人体の両肩中点と機体第2関節pitch軸原点を基準にする |
| `contact-torque`、`safety-torque` | `feedback.py`。同一公称解・入力と固定履歴でMを求め、接触と抵抗を区別して合成 |
| `volume-margin`、`static-allocation` | `wrench_math.h`。固定Qの符号付きXY/XYZ射影余裕と有界共通推力配分 |
| `gate-acceptance`、`rechecked-command` | `shape_runtime.py`。再判定拒否も保持へ移行し、復帰条件を要求 |
| `anchor-compensation`、`bimanual-center-position` | `anchor_runtime.py`。発行関節指令に対する重心位置補償。移動指令の所有者は単一 |
| `anchored-jacobian` | `contact_model.py`。接触Jから固定基準点の並進Jを差し引く |
| `joystick-view-basis`、`joystick-heading` | `navigation.py`。視点と体幹の傾きによる2次元入力面、SO(3)での姿勢積分 |
| `multicontact-torque`、`multicontact-balance` | 個別の接触Jによる力覚合成と、同じ候補形態での合力・重心まわりモーメントの釣合い |

## 設定と起動

- 標準は`mapping_mode:=shape`。`joint`、`dual_shape`、`dual_joint`も選択可能。
- 人体角の順序は肩abd/flex/rot、肘flex、前腕pron、手首flex/dev。
  関節角方式のCAD名・零点・符号尺度・基準姿勢は`shape/joint_mapping`で指定する。
  既定のidentity変換・零基準はシミュレーション用。実時間では測定済み校正を要求する。
- 力評価は`force_dimensions:=2`（XY）が標準。`3`でXYZを選ぶ。
  希望形態と速度制限後の形態で推力偏向角・モデル姿勢を共用する。
- `use_contact_wrench:=true`ではlink1原点まわりの環境→機体レンチを判定に使用する。
  対応frameは`link1`または`dragon/link1`、同じ原点の成分をFC軸で表す場合は`fc`／`dragon/fc`。
  入力断・未知frameは拒否する。非接触時も明示的な零レンチが必要。
- `anchor_mode:=tail`はLink4尾端、`mid`は第2関節pitch軸原点。
  両腕方式では`mid`が標準、片腕方式は互換のため`none`が標準。
  CoG odometryと形態の診断が新鮮なHover中だけ補償し、Joystickによる基準位置更新を同じノードで行う。
- 力覚は`enable_haptics:=true`で有効化し、`haptics`設定でgain・剛性・減衰・上限を指定する。
  接触gainの既定は0。サーボのARMは自動実行せず、実機では[校正・明示ARM](hardware.md)が必要。
- Spinalの配列・送信バッファとホスト検査を14軸対応へ拡張した。
  物理IDは右0〜6・左10〜16、scan indexは別の0〜13。対応する新MCUイメージが必要。

### 視点基準の並進・回転

`navigation/operator_position`にworld座標の操作者基準位置を設定する。
空のままでは視点基準のJoystick速度を出力しない。移動を許す場合は`/dracomancer/operator_position`
（worldの新鮮なPoseStamped）で更新する。体幹IMUの取付補正と`navigation/tilt_alignment`を校正する。
外部計測からの初期化も可能で、動的更新を使い始めた後は計測が古いと入力を零にする。
`require_imu: true`が既定で、IMUが不明・古いときは速度入力を零にする。

片腕は`/dracomancer/joystick/calibrated/pressed`の押下エッジで並進／回転を切り替え、
中立を確認するまで両速度を零にする。両腕は右入力が並進、
`/dracomancer/joystick/left/calibrated`の左入力が回転。左側の実機入力経路・校正は別途用意する。
各瞬間の独立入力は各スティック2軸であり、3軸独立入力とは扱わない。

姿勢目標は回転行列で積分し、速度・姿勢範囲の上限を適用する。
DRAGONの`target_rotation_motion`（CoG対象）を用い、無視されるFlightNavのroll/pitch欄には依存しない。
従来の入力経路は`control_position.py`の`navigation_mode:=legacy`として残す。

### 複数接触

`contacts`に各接触点の`link`・`point`（リンク内座標）・個別の`topic`を並べる。
各レンチのモーメント基準点は設定したpointで、成分はそのリンクまたはFCの向きで表す。
力覚では個別のJを使って合成し、相殺する把持力を合レンチだけにまとめない。

複数接触の可行性判定には、接触固有の摩擦・関節荷重を確認する外部検証器から
`/dragon/contact/constraints_ok`の新鮮なBool=trueも要求する。
この検証器や摩擦係数・関節負荷モデルは本変更で仮定して作らず、未設定時は形態更新を拒否する。
全体の静的釣合いだけで把持を認証しない。

## 検証範囲

数値テストは式の対応と異常入力時の保持を確認する。実機での安全値、通信周期、飛行安定性、
力覚の受動性、基準点の保持精度は別途の検証を要する。
有界配分は残差で成立を確認し、計算が収束しない場合は保守的に拒否する。
固定した推力方向の射影領域が退化すると、正の余裕は得られない。従来の可変偏向角による余裕や
閾値と同等とは扱わず、起動形態・評価角・XY/XYZ別に検証する。
サンプル間の可行性や動的安定性を証明する判定ではない。
論文の第5・6章は結果未記入であり、ソフトウェアテストを実機実証として記載しない。

検証済みはPython回帰テスト66件、CAD整合性、Python構文、launchのXMLとローカル引数、
文書リンク、およびWindows上でビルド・実行したC++数値計算／Spinal通信の単体テスト。
ROSを模擬したテストを含み、PyKDL照合、Catkin全体、Gazebo、STM32ファームウェア全体の
ビルド・実行はこの環境では未検証である。

変更適用には`catkin build spinal dracomancer`と新しいサービス型による依存クライアントの再起動が必要。
Spinalはさらに適切なMCUターゲットでビルドする。実機への書き込みは本作業では行わない。
