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
未校正で動きだけを見る場合は`roslaunch dracomancer observe.launch`で読み取り専用の仮表示を起動する。
既定はID 1〜7の右腕で、起動姿勢を0度とする。[仮表示の条件](docs/hardware.md#未校正の動作確認だけを行う場合)を参照する。
既定は`calibrated: false`・空の関節対応表で、サーボを駆動しない。
`mkii.launch`も同じ引数で利用できる。

| 場所 | 内容 |
|---|---|
| `urdf/cad.urdf`、`urdf/mkii.urdf.xacro` | CAD基準モデルとシミュレーション用モデル |
| `meshes/` | 新STL 8種類。旧STLは使用しない |
| `config/mkii/` | CAD検証情報、14関節の対応・制御・実機校正設定 |
| `config/plotjuggler.xml` | 力・トルク余裕のPlotJuggler表示設定 |
| `scripts/mkii/` | viewer・Gazebo・直結制御とGUI |
| `src/dracomancer/` | 校正済み電流換算・セッション・出力排他のPython部品 |
| `scripts/control/shape_*.py` | CAD中心線による形状写像・診断 |
| `test/` | 単体・回帰テストとROS・Gazebo検証。Mk-II固有の検証は`test/mkii/` |
| `figures/scripts/` | オフラインの作図スクリプト |
| `figures/archive/` | 保存対象とした過去の図・CSV・集計。現行Mk-IIの検証結果とは区別する |

新規のログ・bag・作図結果は一時生成物としてGit管理から除外する。
作図スクリプトの出力先は`figures/`内または実験結果の保存先とし、`figures/scripts/`へ混在させない。
保存対象と決めた成果物のみ`figures/archive/`へ移し、通常の出力先にはしない。

文書の入口は[ドキュメント一覧](docs/README.md)。
起動・検証は[操作手順](docs/usage.md)、研究仕様・未実装の課題は一覧内の各設計書を参照する。
