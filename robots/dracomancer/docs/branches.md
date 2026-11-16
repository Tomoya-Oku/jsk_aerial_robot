# Dracomancerブランチ調査

本書はブランチ統合時点の履歴。後続のMk-IIパッケージ統合・Mk-I削除に伴う現行構成は
[パッケージ統合記録](integration.md)を参照する。

## 統合結果（2026-11-17）

- `feat/shape-mapping`をfast-forwardで取り込み、`feat/mkii-haptics`と
  `feat/contact-haptics`をマージした。`feat/mkii-urdf-torque`は既存の統合内容を維持し、再マージしていない。
- 指示者の承認に基づき、shape設定と接触Jacobian出力を両方残してコンフリクト5ファイルを解消した。
  設計書は汎用力覚と壁押し実験の条件を区別して併記した。
- 力覚出力を持つ後発ノードは共通ロックの取得に失敗すると出力前に停止する。
  ロック競合時には動作中ノードへ0電流・OFFを送らない。計算専用ノードは併用できる。
- shapeでは既存接触力覚用`MappingState`の配信と壁押し力覚ノードを無効にした。
  汎用力覚もshape指定時は接触成分・実機出力を無効にし、未対応のJacobianを実機へ使わない。
  起動条件・排他の適用範囲は[ソフトウェア設計5.7.6](design-spec/05-software.md#576-統合後の起動排他とshapeモード)を参照。
- 差分は `robots/dracomancer/` 内に限定した。Spinal・Neuronの実装は変更していない。

### 今回の検証

Windows / Python 3.11の隔離環境で、以下の合計40テストが成功した。

| 検証対象 | 件数 | 実行対象 |
|---|---:|---|
| 接触力覚・従来写像・計測時刻 | 10 | `test/test_contact.py` |
| Mk-II力覚計算・制限・電流換算 | 7 | `scripts/haptics/test_haptics.py` |
| 二重出力拒否・終了後の再取得・shape無効化 | 8 | `test/test_output.py` |
| 実URDFの中心線・最適化・関節制限 | 15 | `tests/test_shape.py`の`ShapeTests`からKDL照合を除いたもの |

形状テストは実際のMk-II・DRAGON xacroを使用した。単独xacro wheelのROS 2向け
`$(find)`解決だけを、実パッケージを探索する`rospkg.RosPack().get_path`へ差し替えて実行した。
さらに、変更Python 25ファイルとXML 6ファイル（launch・package.xml）の構文、
4写像モードの起動条件、依存の重複、`git diff --check`、未解消コンフリクトがないことを確認した。

ROS/Catkin環境・Gazebo・PyKDLは利用できず、Catkinビルド、ROS依存の停止監視・形態制御テスト、
Gazebo統合・実機試験は未実施。今回の排他実行試験はWindows側で実施し、Linuxのflock分岐は未実行。
既存ブランチ文書に記載された過去のROS/Gazebo試験結果を、今回の再検証結果とは扱わない。

適用時は **`catkin build dracomancer dracomancer_mkii_ros`による再ビルドが必要**。
`MappingState.msg`の生成とGazebo接触プラグインのビルドを含む。

以下の一覧・統合状況は初回調査時点の記録。

## 初回調査

調査日：2026-11-17（JST）。GitHubのブランチ一覧とローカルのコミット・実装差分を照合した。
比較基準は `develop`（`abbc6ddb`）。ファイル数は共通祖先から各ブランチ先端までの
`git diff develop...<branch>` によるもので、現在のdevelopに未統合のファイル数とは限らない。
ローカルにある同名ブランチは、対応するorigin参照と同じ先端だった。

## GitHubに現存する4ブランチ

以下のブランチ名にはすべて `dracomancer/` が付く。

| ブランチ | 先端 | 差分ファイル数 | 実装内容 | developへの統合状況 |
|---|---|---:|---|---|
| `feat/contact-haptics` | `bcde2d48` | 24 | GazeboのDRAGON先端で壁を押し、接触反力をDracomancerへ返す実験系。接触センサ用プラグイン、実行時の写像ヤコビアン、重心基準のロボットヤコビアン、既存Spinalトピック経由のXM430電流出力を追加。明示的な有効化と入力・物理時計の停止監視も実装。 | 未統合 |
| `feat/mkii-haptics` | `9a5df8a2` | 17 | Mk-II向けの汎用的な力覚提示基盤。接触wrenchを `Mᵀ Jᵀ w` で人体7関節のトルクへ変換し、形状補正による抵抗・減衰と合成。XM430電流換算、トルク・変化率制限、起動系への接続を追加。 | 未統合 |
| `feat/mkii-urdf-torque` | `dcf6a07a` | 54 | CAD由来のMk-II両腕モデル（15リンク・14関節）、専用Catkinパッケージ、RViz/Gazebo、角度・トルク操作GUIを追加。Spinal側にはUART直結XM430の電流制御・モード切替・指令期限監視を実装。 | PR #18で統合済み。先端ツリーはdevelopと完全一致 |
| `feat/shape-mapping` | `6cd6940a` | 15 | Mk-II右腕とDRAGONのURDFから中心線を計算し、長さを正規化して全体形状を数値最適化で写像。既存の可行性ゲート、関節角・角速度制限、診断・可視化、Gazebo操作デモと検証を追加。 | 未統合。現在のdevelopから派生 |

`contact-haptics` は壁押し実験を一式で動かす実装、`mkii-haptics` は接触・安全力覚の
計算と出力を部品化した実装である。両者は同じ基点 `011f9856` から独立して分岐しており、
片方がもう片方を取り込んだ関係ではない。`mkii-urdf-torque` も同じ基点から派生していた。

## ローカル参照だけが残る2ブランチ

GitHubの現行一覧には存在しない。リモート追跡参照はpruneしていない。

| ブランチ | ローカルでの残存場所 | 内容・状態 |
|---|---|---|
| `dracomancer/chore/organize-for-mkii` | origin追跡参照 | Mk-II仕様書を10章に分割して正本化し、Mk-Iの旧README・図・実験生成物を整理。ログ可視化・メディア出力の仕様とエージェント指示も更新。PR #17で統合済み。 |
| `dracomancer/mkii-design-spec` | ローカルブランチとorigin追跡参照 | 上記と同じ `298bb178` を指す別名で、固有差分なし。 |

両ブランチの先端ツリーはPR #17の取り込みコミット `011f9856` と完全一致した。
取り込みはsquashのため、コミットの祖先判定やahead数だけでは統合済みと判定できない。

## 確認範囲

- GitHub APIの現行ブランチ名・先端SHAと、ローカル参照を照合。
- 共通祖先、コミット履歴、変更ファイル一覧、主要なコード・launch・設定・文書を調査。
- 統合済みの2系統は取り込みコミットとのツリー差分が空であることを確認。
- 本調査ではROS・Gazebo・実機試験を再実行していない。実装内容の整理であり、動作保証ではない。
- 変更は本書のみ。Catkinによる再ビルドは不要。

参照：[GitHubブランチ一覧](https://github.com/Tomoya-Oku/jsk_aerial_robot/branches)、
[PR #17](https://github.com/Tomoya-Oku/jsk_aerial_robot/pull/17)、
[PR #18](https://github.com/Tomoya-Oku/jsk_aerial_robot/pull/18)。
