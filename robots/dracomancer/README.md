# Dracomancer Mk-II

XM430-W350-Rを用いるMk-IIのCADモデル、新STL、角度・トルクGUI、Spinal UART/RS485直結制御、
DRAGONの形状操作を収録するCatkinパッケージ。標準名前空間は`/dracomancer`。
Mk-Iと独立パッケージ`dracomancer_mkii_ros`は統合・廃止した。

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
既定は`calibrated: false`・空の関節対応表で、サーボを駆動しない。
`mkii.launch`も同じ引数で利用できる。

| 場所 | 内容 |
|---|---|
| `urdf/cad.urdf`、`urdf/mkii.urdf.xacro` | CAD基準モデルとシミュレーション用モデル |
| `meshes/` | 新STL 8種類。旧STLは使用しない |
| `config/mkii/` | CAD検証情報、14関節の対応・制御・実機校正設定 |
| `scripts/mkii/` | viewer・Gazebo・直結制御とGUI |
| `src/dracomancer/` | 校正済み電流換算・セッション・出力排他のPython部品 |
| `scripts/control/shape_*.py` | CAD中心線による形状写像・診断 |

文書の入口は[ドキュメント一覧](docs/README.md)。
起動・検証は[操作手順](docs/usage.md)、研究仕様・未実装の課題は一覧内の各設計書を参照する。
