# jsk_aerial_robot (fork) Instruction

- このファイルはAIエージェントに対する，jsk_aerial_robotリポジトリ全体の指示書である．
- AIエージェントはまずこのファイルを確認した後，作業対象のディレクトリにREADME.mdやAGENTS.md，CLAUDE.mdが存在するかどうかを必ず確認し，存在する場合はそれらに従う．

## 基本ルール

- スクリプトやlaunchファイルなど，新しくファイルを作る際には**簡潔でひと目見て分かりやすい**ファイル名をつける．基本的に3単語以上を"_"や"-"でつないだ長いファイル名は避ける．
- 変更は最低限とし，パッケージを跨いだ変更は可能な限り避ける．
  - 例えば，dracomancerとaerial_robot_nerveを同時に変更することはできるだけ避ける．
  - どうしてもパッケージを跨いだ変更が必要な際には提案し，指示者の確認を取る．
- 基本的に元の仕様（引数など）は以後も使用できるようにする．
- pythonスクリプトやlaunchファイルなどの引数を最低限にし，デフォルト（無入力）値は標準的な入力値とする．
- 指示者が似たような指示を繰り返し行っている場合は，AGENTS.mdへの追加やスキル作成を提案する．
- グラフ作成用のスクリプトファイルはROS用のscripts/に混ぜず，robots/<robot_name>/figures/内に入れる．
- 重要なレポート・レビューなどはrobots/<robot_name>/docs/内にMarkdown形式で保存する．
- 実装を変更した場合には，各ドキュメントファイルの変更が必要でないかを必ず確認し，修正が必要であれば修正する．
- 単純なもの以外、基本的にコードの実装の際に意図などのコメントをつける。
- 変更の適用にはCatkinによる再ビルドの要否を最後に必ず報告する

## ブランチ

このリポジトリでは以下のルールに従ってブランチを運用する．

- mainは[upstream](https://github.com/jsk-ros-pkg/jsk_aerial_robot) のmasterブランチをpullする専用のブランチとする．
- developが開発の主軸ブランチとする．
- 現在は一時的にdevelopブランチの保護ルールを外している．直接developにプッシュして作業を進める．
<!-- - 作業は最新developをベースにブランチを切って行う． -->
- ブランチ名は`<作業対象>/<タグ>/<作業内容>`とする．以下は良い例．
  - dracomancer/feat/haptics-feedback-test
  - spinal/feat/m5-stack-joystick
  - dragon/chore/publish-fc-radius
- 作業対象は基本的にロボット名とし，aerial_robot_*の変更の場合は以下のようなルールに従う．
  - aerial_robot_base: base
  - aerial_robot_control: control
  - aerial_robot_estimaion: estimation
  - aerial_robot_model: model
  - aerial_robot_msgs: msgs
  - aerial_robot_nerve: nerve
  - aerial_robot_simulation: simulation
  - aerial_robot_web: web
- 例外として，spinalやneuronなどの具体名がある場合はそれを用いる．

## コミット

このリポジトリでは以下のルールに従ってコミットを行う．

- コミットメッセージは`[<作業対象>] <タグ>: <コミットメッセージ本文>`とする．以下は良い例．
  - [Dracomancer] feat: 力覚提示のテストスクリプトを追加
  - [Spinal] feat: M5-stackジョイスティックのライブラリファイルを追加
  - [Dragon] chore: /dragon/debug/fc_f_minと/dragon/debug/fc_t_minをpub
- 作業対象は基本的にロボット名とし，aerial_robot_*の変更の場合は以下のようなルールに従う．
  - aerial_robot_base: Base
  - aerial_robot_control: Control
  - aerial_robot_estimaion: Estimation
  - aerial_robot_model: Model
  - aerial_robot_msgs: Msgs
  - aerial_robot_nerve: Nerve
  - aerial_robot_simulation: Simulation
  - aerial_robot_web: Web
- 例外として，spinalやneuronなどの具体名がある場合はそれを用いる．
- タグは以下から適切なものを選択する．
  - feat: 新機能の追加
  - chore: 微小な変更
  - fix: 大規模な変更
  - refactor: 挙動に変化のない変更
  - ci: Github Actions関連の編集
  - doc: ドキュメントの追加・変更
  - agent: ドキュメントの追加・変更の内，AIエージェントに対する指示ファイルに関わるもの（AGENTS.mdなど
- コミットメッセージ本文は，特に指示がなければ日本語で記述する．

## PR作成・更新

このリポジトリでは以下のルールに従ってPR作成・更新を行う．

- PR先はdevelopとする．
- PRタイトルは`[<作業対象>] <タグ>: <PRタイトル本体>`とする．以下は良い例．
  - [Dracomancer] feat: HYDRUS XIに対応
- 作業対象・タグの書式は「コミット」を参照する．
- 修正系のタグの場合，PRタイトル本文は「現在の状態→PRによる修正」で挙動がどう変わるのかを明確に書く
- PR説明文は以下の要素を必ず含む．
  - 背景・目的: PRによる修正の理由
  - 修正内容（技術的）: PRによる修正をROSトピック名などを用いて技術的に説明．箇条書きを用いる．
  - 修正内容（簡潔）: PRによる修正を非エンジニアにも分かるレベルで簡潔に説明．箇条書きを用いる．
  - 確認事項: 検証した事項を箇条書きで書く．
  - 未確認事項: 残る課題や未検証の事項を箇条書きで書く．
  - 補足: その他の注意書きなど．
- PR作成後にそのブランチにプッシュが合った場合には必ず元のPR本文を確認し，相違・追加があれば更新する．
