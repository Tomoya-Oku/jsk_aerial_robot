# Mk-IIパッケージ統合

## 確定した方針

- `dracomancer_mkii_ros`のCADモデル・新STL・操作GUI・直結制御を`dracomancer`へ集約する。
- Mk-Iを削除し、標準名前空間を`/dracomancer`、関節名をCADの`right_joint_1..7`等に統一する。
- 実機の標準経路はSpinal UART/RS485直結。未校正時の駆動拒否と指令期限監視を維持する。
- shapeと接触力覚の接続は未実装のままとし、推定の関節対応やトルク校正値を追加しない。

## 削除範囲の確認

以下はすべてGitで管理されている実装で、既存コミットから復元できる。
自動承認レビューによる一括削除の拒否後、指示者が以下の1〜3を明示承認したため削除した。

### 1. Mk-Iモデル・制御・入出力

`robots/dracomancer/`配下の次のファイル・ディレクトリ。

- `urdf/dracomancer.urdf`、`urdf/mesh/`：旧CADと旧STL。
- `src/control/`、`src/model/`、`src/dracomancer.cpp`、`include/`、`plugins/`：旧機体用C++制御・モデル。
- `config/Servo.yaml`、`config/RvizInit.yaml`、`config/rviz_config`：旧関節名・表示設定。
- `config/state_estimation.yaml`、`config/simulation.yaml`、`config/params.yaml`、`config/navigation.yaml`、
  `config/control/control.yaml`、`config/control/optimizer.yaml`、`config/sensors/`：旧機体構成。
- `launch/include/`：旧機体用センサ・サーボ起動。Spinal bridgeは新bringupから必要時に起動する。
- `scripts/device_io/convert_servo_to_joint_states.py`、`scripts/device_io/publish_fake_joint_states.py`：旧関節名の入出力。
- `scripts/visualization/`、`scripts/control/control_orientation.py`：旧関節ラベル・旧姿勢操作。

### 2. Mk-I関節名を前提とするCAN力覚経路

UART直結制御は`dracomancer`へ移設した`direct.py`を使う。以下の旧経路を削除する場合も、
一般的な接触・安全力覚の計算部品`contact_feedback.py`・`safety_feedback.py`と排他ロックは残す。

- `launch/haptics.launch`、`config/haptics.yaml`。
- `scripts/control/contact_haptics.py`。
- `scripts/haptics/haptic_controller.py`、`scripts/haptics/servo_output.py`、`scripts/haptics/current_conversion.py`。
- `msg/MappingState.msg`：旧解剖学的関節写像用のJacobian出力。
- `test/test_haptics.py`、`test/teleop_smoke.py`：旧CAN力覚用の試験。
- `test/test_contact.py`・`test/test_output.py`・`scripts/haptics/test_haptics.py`の旧経路専用テストを整理。
  接触数理、直結電流制御、CAD形状写像、排他・入力監視のテストは残すか移設する。

### 3. 移設後の旧パッケージ

- `robots/dracomancer_mkii_ros/`の残りのビルド設定・package.xml・README等。
  URDF・STL・設定・Python実装・試験・実機手順は`robots/dracomancer/`へ移設済み。

## 検証・適用

新STL 8種類のSHA-256、CADの15リンクの慣性・14関節の変換／軸、14個のtransmissionを確認した。
元CADのハッシュを`config/mkii/cad.json`の`source_urdf_sha256`に保持し、
パッケージURI・robot名だけを変えたCADのハッシュを`baseline_urdf_sha256`として記録する。

Windows / Python 3.11の隔離環境で次の51テストが成功した。

| 検証 | 件数 |
|---|---:|
| 直結電流換算・モード読戻し・機種／校正拒否 | 4 |
| 表示と計測の分離・元計測時刻・未校正拒否・排他・clutch・速度制限・インストール経路 | 10 |
| 出力ロックの競合拒否・プロセス終了後の解放 | 2 |
| 接触wrenchと重心基準の数理 | 5 |
| 汎用接触・安全力覚の計算 | 5 |
| 実モデルによる中心線・最適化・関節境界 | 15 |
| 形態制御の入力監視・Hover・可行性ゲート・速度制限 | 10 |

ROSインターフェースは模擬した。形状テストの単独xacro wheelでは`$(find)`のみを
`rospkg.RosPack().get_path`へ接続して実パッケージを解決した。
Python 51ファイルの構文、launch 10ファイルの引数・include・資源参照、package.xmlの検証、
Python部品の一時領域へのインストールとimport、新旧CADの慣性・軸・変換の一致も確認した。
ROS/Catkinビルド、実際のROS・Qt・Gazebo、PyKDL照合、Linux flock分岐、実機の物理動作は未確認。
過去のブランチでの試験成功と今回の検証を区別する。

適用には **`catkin build spinal dracomancer`による再ビルドが必要**。
既存workspaceでは旧パッケージと削除済みメッセージ・プラグインのビルド生成物を残さず、
Catkinのclean機能で整理してからビルドし直す。
標準起動は`roslaunch dracomancer bringup.launch`（viewer）。旧パッケージ名やMk-Iのrm/sim起動は廃止した。
Mk-Iの過去ログを読む図生成・解析資料は履歴資料として保持し、実機制御の経路には接続しない。
