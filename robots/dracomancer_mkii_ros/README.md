# Dracomancer Mk-II ROS 1

CAD由来のURDF、STL、Gazebo Classic制御、RViz、角度／トルク子ウィンドウを収録するCatkinパッケージ。
既存の `dracomancer` はMk-Iとして保持し、Mk-IIは専用の名前空間 `/dracomancer_mkii` を使う。

```bash
catkin build spinal dracomancer_mkii_ros
source devel/setup.bash
roslaunch dracomancer_mkii_ros view.launch
roslaunch dracomancer_mkii_ros gazebo.launch
```

既存パッケージからは `roslaunch dracomancer mkii.launch backend:=gazebo` でも起動できる。
既定の `backend:=viewer` はRVizの表示と角度操作のみ。

| ファイル／ディレクトリ | 内容 |
|---|---|
| `urdf/cad.urdf` | CADから生成した15リンク・14関節の基準モデル。メッシュURIだけを本パッケージへ変更 |
| `urdf/mkii.urdf.xacro` | 基準モデルにworld固定、transmission、仮の制御設定と箱近似collisionを追加 |
| `meshes/` | メートル単位の8種類のSTL |
| `config/cad.json`, `joint_mapping.csv` | CAD出力のメッシュhash・検証情報と関節対応 |
| `scripts/panel.py` | Mk-Iと同じjoint_state_publisher_guiを再利用した角度／トルク画面 |
| `scripts/control.py` | viewer、Gazeboの位置／トルク切替と期限監視 |
| `scripts/direct.py`, `config/hardware.yaml` | Spinal UART直結用。校正と対応表が未設定なら駆動不可 |

実機対応は [docs/hardware.md](docs/hardware.md)、リポジトリへの組込みと検証手順は
[既存Dracomancerの手順書](../dracomancer/docs/mkii.md) を参照。
研究の正本は [ハードウェア設計](../dracomancer/docs/design-spec/04-hardware.md) と
[ソフトウェア設計](../dracomancer/docs/design-spec/05-software.md)。

CADには左右両腕が含まれる。continuous関節、CADのq=0、2Nm・2rad/sの設定、PID、
減衰と箱collisionはシミュレーション用の仮設定で、実機の可動限界や保証値ではない。
自己衝突は無効。実機への書込み・物理的なトルク／停止は未確認。
