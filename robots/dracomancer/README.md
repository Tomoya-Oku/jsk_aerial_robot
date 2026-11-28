# Dracomancer Mk-II

XM430-W350-Rを用いるMk-IIのCADモデル、新STL、角度・トルクGUI、Spinal UART/RS485直結制御、
DRAGONの形状操作を収録するCatkinパッケージ。標準名前空間は`/dracomancer`。
Mk-Iと独立パッケージ`dracomancer_mkii_ros`は統合・廃止した。
制御理論は修士論文を正本とする。[対応表と設定](docs/thesis.md)を参照する。

```bash
catkin build spinal dracomancer
source devel/setup.bash

# 標準：RVizと角度スライダ。実機には接続しない。
roslaunch dracomancer bringup.launch
# Mk-II単体の物理シミュレーションと角度・トルク操作
roslaunch dracomancer bringup.launch backend:=gazebo
# DRAGONとの形状操作デモ（離陸しない）
roslaunch dracomancer shape_demo.launch
```

実機は`backend:=hardware`を指定し、[直結制御の手順](docs/hardware.md)に従う。
未校正で動きだけを見る場合は`roslaunch dracomancer bringup.launch backend:=preview`で読み取り専用の仮表示を起動する。
既定はID 1〜7の右腕で、起動姿勢を0度とする。[仮表示の条件](docs/hardware.md#未校正の動作確認だけを行う場合)を参照する。
既定は`calibrated: false`・空の関節対応表で、サーボを駆動しない。
各launchの用途は[起動一覧](docs/usage.md#launch一覧)を参照する。

| 場所 | 内容 |
|---|---|
| `urdf/dracomancer.urdf`、`urdf/dracomancer.urdf.xacro` | 2026年11月27日版のCAD基準モデルと、同じ形状・関節を使うROS/Gazebo用モデル |
| `meshes/` | CAD基部・左右共用の腕部品・左右別の手首部品を収めるSTL 10点 |
| [`config/`](docs/usage.md#設定ファイル一覧) | 関節、実機校正、制御、可視化、実験の設定 |
| `scripts/device/` | Spinal直結、計測プレビュー、Joystick入力・校正、関節操作GUI |
| `scripts/teleop/` | DRAGONの関節角・位置・姿勢の操作 |
| `scripts/haptics/` | 力覚フィードバック |
| `scripts/safety/` | 力・トルク余裕のフィルタと監視 |
| `scripts/simulation/` | viewer／Gazeboの関節制御、地上デモの入力、接触壁 |
| `scripts/control/` | 形態写像・接触計算・出力排他を共用するPythonモジュール |
| `src/feasibility/` | 候補形態の可行性を評価するROSサービス |
| `src/simulation/` | Gazeboの壁接触プラグイン |
| `figures/scripts/` | オフラインの作図スクリプト |
| `figures/YYYYMMDD_HHMMSS_名前/` | 保存対象の図・CSV・集計・録画 |
| `rosbags/YYYYMMDD_HHMMSS_名前.bag` | ローカルのROS bag記録 |

## 保存済みの実験結果

フォルダ名の日時にはデータの取得日時を使う。取得日時を確認できなかったものは、表に記載した日時を使った。

| フォルダ | 内容 | 日時の根拠 |
|---|---|---|
| [`20260630_225241_fc_thresholds_4096grid`](figures/20260630_225241_fc_thresholds_4096grid/) | 旧計算による4096形状の力・トルク余裕 | 最初のSVGの作成日時 |
| [`20260706_175417_shape_test`](figures/20260706_175417_shape_test/) | 形状操作試験の図・PDF・集計 | ROS bag名 |
| [`20260708_151650_joint_angles_mapping`](figures/20260708_151650_joint_angles_mapping/) | 関節写像と位置の図・集計 | ROS bag名 |
| [`20260708_185555_rsj2026_test`](figures/20260708_185555_rsj2026_test/) | RSJ 2026向けの解析図 | ROS bag名 |
| [`20260724_153451_fc_thresholds_4096grid_signed`](figures/20260724_153451_fc_thresholds_4096grid_signed/) | 符号付き計算による4096形状の力・トルク余裕 | 最初のSVGの作成日時 |
| [`20261127_144120_shape_demo`](figures/20261127_144120_shape_demo/) | 両腕の形状マッピングデモの録画2本 | 1本目のGit登録日時 |

`figures/20260621_223025_fc_experiment/`の集計に対応する記録は`rosbags/20260621_223025_fc_sweep.bag`。bag内の時刻はシミュレーション時刻のため、内容が一致するコピー元bagの最終書き込み時刻を使った。用途名が付いていなかったbagには`recording`を付けた。

ログ・bag・再生成した図は一時生成物として扱う。再生成先には`figures/`内の別フォルダか外部の結果保存先を指定する。
Gitで保存する成果物はフォルダごとに`.gitignore`の除外対象から外す。

文書の入口は[ドキュメント一覧](docs/README.md)。
[スクリプト一覧](docs/software.md#スクリプト一覧)と[C++ソース一覧](docs/software.md#cソース)にファイルごとの役割を記載する。
起動・検証は[操作手順](docs/usage.md)、研究仕様・未実装の課題は一覧内の各設計書を参照する。
