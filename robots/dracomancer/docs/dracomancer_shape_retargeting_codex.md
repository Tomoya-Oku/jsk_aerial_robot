# Codex実装指示書：Dracomancer → DRAGONのCross-embodiment Shape-based Retargeting

## 0. 目的と作業方針

`Tomoya-Oku/jsk_aerial_robot` の `develop` を基準に、Dracomancer（7関節自由度）の人腕形状を**共通の3D中心線形状空間**で表現し、DRAGON（4リンク、3関節×Pitch/Yaw＝内部6DoF）の目標形態へ変換するモード `mapping_mode:=shape` を実装してください。

**最初の完成目標は「形状マッピング単体＋既存の安全ゲートへの接続＋シミュレーションでの比較可能性」です。** 新しい飛行制御器、独立した安全サービス、機械学習モデル、ROS 2移行、実機での自動飛行は今回の対象外とします。

- リポジトリ直下の `AGENTS.md`、`robots/dracomancer/AGENTS.md`、`CLAUDE.md`、対象ディレクトリのREADMEを**先に読む**こと。
- 正本は `robots/dracomancer/docs/design-spec.md` および `docs/design-spec/` の各章。特に `03-concept.md`、`05-software.md`、`06-evaluation.md`、`08-priorities.md`、`09-open-issues.md` を読むこと。既存仕様と矛盾した場合は黙って仕様を変更せず、差分を報告すること。
- 変更は原則 `robots/dracomancer/` 内に限定。`dragon/` や `aerial_robot_*` の変更が必要なら、**実装前に変更案を提示して確認を求める**こと。
- 既存 `distal`、`joint_pairing`、`geometric` の挙動を維持し、デフォルト `distal` を変更しないこと。`shape` は明示的に指定された場合のみ有効化する。
- 既存のROS 1トピック・メッセージ・安全ゲート・実験ログを可能な限り再利用する。ファイル名は短く、変更箇所は小さく保つ。
- 推測で関節順、座標軸、リンク長、仮想基準点、安全しきい値を確定しない。コード・URDF・TF・正本を照合し、未決定値は設定可能にすること。
- 実装の各段階でテストを実行し、最後に変更ファイル、動作確認、未確認事項、Catkin再ビルドの要否を報告すること。

## 1. 既存コードの調査（変更前の必須作業）

以下を読んで現在のデータフローと既存機能を理解してください。

| 調査対象 | 着眼点 |
|---|---|
| `scripts/control/control_joint_angle.py` | `mapped_target()` / `geometric_target()` / `teleop_shape_target()` / `rate_limit()` / `make_joint_msg()` の呼出順序、40 Hz動作、ゲートの再評価間隔 |
| `launch/teleoperation.launch` | `mapping_mode`、JointState入力、安全ゲート、リンク4アンカーの引数 |
| `urdf/dracomancer.urdf` | 7関節の順序・原点・回転軸・固定リンク・手先定義 |
| `scripts/shape_task/dragon_fk.py` | DRAGONの既存簡易FK、link4基準の点列、リンク長・関節オフセット |
| `scripts/shape_task/README.md` と `config/shape_task/` | 既存の実験比較・ログ・シミュレーション実行方法 |
| `src/shape_feasibility_node.cpp` と `srv/ShapeFeasibility.srv` | `check_shape` の返値、`allocation` モード、および予測上の限界 |
| DRAGONのURDF/TF/robot model | 既存の簡易FKが実際のリンク形状・関節順序に一致するか |

確認時の既知情報：

- 人体側関節名は `shoulder_abduction_adduction_joint`、`shoulder_flexion_extension_joint`、`upper_arm_external_internal_rotation_joint`、`elbow_flexion_extension_joint`、`wrist_supination_joint`、`wrist_flexion_extension_joint`、`wrist_abduction_adduction_joint`。**添字で並べ替えずJointState.nameに基づいて取得**する。
- DRAGON内部関節名は `joint1_pitch, joint1_yaw, joint2_pitch, joint2_yaw, joint3_pitch, joint3_yaw`。DRAGONの中心線は操作者側と対応させるため、**link4側からlink1側**を正方向とする。
- 現行コードには `geometric` モードがあるが、これは主にリンク方向をYaw/Pitchに分解する方式であり、**全中心線を共通形状空間で最適化する方式とは異なる**。今回の `shape` と混同しない。
- 既存 `shape_task/dragon_fk.py` は実験用簡易モデルなので、そのまま物理的に正しいと仮定しない。DRAGONのURDF/TF上の代表姿勢と比較して検証する。
- 現在の仕様では**腕の全体移動・全体回転をDRAGONのbase poseには用いない**。base poseはJoystick等が担当する。以前の案の「肩3DoF→DRAGON機体姿勢」は採用しない。

## 2. 数理仕様：共通形状空間

### 2.1 入出力

\[
\boldsymbol q_H\in\mathbb R^7,\qquad
\boldsymbol q_D\in\mathbb R^6.
\]

DracomancerのURDF順運動学から**固定した胴体・装着基準座標系**で人体リンク点列を計算する。最低限、肩・肘・手首・手先の解剖学的な代表点を使う。DRAGONも固定したlink4基準で、根元から末端へ点列を生成する。

**重要な修正事項：** 人体の中心線を「動いている上腕や第1可動リンクの姿勢」で回転正規化してはいけない。肩の方向操作が消えるからである。取り除くのは世界座標系上の装着位置・固定取付姿勢だけであり、**肩関節による相対回転は必ず保持**する。

現行設計書はDRAGON Link4に対応する「肩～体幹の仮想リンク」を要求しているため、優先案は次の5点を使った4セグメント構成とする。

\[
P_H=[p_{\mathrm{torso}},p_{\mathrm{shoulder}},p_{\mathrm{elbow}},p_{\mathrm{wrist}},p_{\mathrm{hand}}].
\]

ただし `p_torso` の定義は**正本の未確定事項**である。URDFの `base_link`・肩関節位置・取付寸法から推測して黙って固定値を採用しない。設定ファイルで指定できるようにし、未設定時は誤った4セグメント写像を実行せず、診断を出すこと。3セグメント（肩→手先）でのアルゴリズム単体テストは可だが、肩形状の同等性を主張しない。

### 2.2 中心線サンプリング

点列を結ぶ**区分線形中心線**を構成し、各線分の長さに基づく累積弧長で、\(s_k=k/(K-1)\ (k=0,\ldots,K-1)\) に等弧長サンプリングする。初期値は \(K=20\) とする（設定可能）。

\[
\hat p_X(s_k)=R_X^{\mathrm{align}}\frac{p_X(s_k)-p_X(0)}{L_X},\quad
z_X=[\hat p_X(s_0)^T,\ldots,\hat p_X(s_{K-1})^T]^T\in\mathbb R^{3K}.
\]

- \(X\in\{H,D\}\)、\(L_X\) は各中心線の**全長**（ゼロ長・NaNを拒否）、\(R_X^{\mathrm{align}}\) は**固定のキャリブレーション回転**。動く関節の姿勢を使わない。
- 人体とDRAGONで、中心線の開始点、左右・前後・上下軸、手先方向、仮想リンクの基準を統一する。キャリブレーション初期値を機械的な事実と混同しない。
- 点列の物理リンク端点とサンプリング点を両方保持してデバッグ表示できるようにする。
- 高次元化は**形状の表現次元を増やすだけ**であり、操作者の独立自由度が増えるわけではない。
- 純粋な軸回旋は中心線に反映されない場合がある。初期版ではこの情報欠落を明記し、手先姿勢・リンク局所フレームの追加は拡張課題とする。

### 2.3 目標関節角の最適化

定義：

\[
\begin{aligned}
E_{\mathrm{shape}}(q)&=\frac{1}{\sum_k w_k}\sum_{k=0}^{K-1}w_k\left\|\hat p_D(s_k;q)-\hat p_H(s_k;q_H)\right\|_2^2,\\
q_{\mathrm{ref}}&=\arg\min_{q\in\mathcal Q_{\mathrm{joint}}}\left[E_{\mathrm{shape}}(q)
+\lambda\|q-q_{\mathrm{prev}}\|_2^2
+\rho\|q-q_{\mathrm{rest}}\|_2^2\right].
\end{aligned}
\]

- `q_prev` は前回の**採用済みコマンド**。`q_rest` は明示的に設定された参照形態（例：startup pose）。\(\lambda,\rho,w_k\) は設定値として保存し、初期値の意味を文書化する。
- \(\mathcal Q_{\mathrm{joint}}\) はDRAGONの実際のURDF/設定に基づく角度制限。各軸一律の±90度を検証なしにハードコードしない。
- **まずは位置形状誤差のみを目的関数とする**。接線、曲率、手先姿勢、障害物距離は任意の拡張に留める。
- 毎周期のwarm start（前回の解）、有限差分の形状ヤコビアン（`3K × 6`）、減衰付き最小二乗／Gauss–Newton、関節境界への投影、バックトラッキング（改善するステップのみ採用）を使う。追加ライブラリは原則増やさず、まずNumPy中心に実装する。
- 更新式の例：\(J=\partial z_D/\partial q\)、\(e=z_H-z_D\)、\(\Delta q=(J^T WJ+\lambda I)^{-1}J^TWe\)。ただし正則化項に `q_rest` を入れる実装では対応する勾配を追加し、上式を最終目的関数の厳密な解と誤認しない。
- 時間予算／最大反復数／収束しきい値／失敗時の保持動作を設定可能にする。異常値・最適化失敗・時間超過時は**最後に検証された指令を保持**し、ゼロ角や未検証の解を無条件に送らない。
- 初期版ではK=20、最大反復を小さく抑え、40Hz制御ループ内で測定する。目標は**1周期25ms未満**（測定環境を記録）。達成困難なら前回解を保持しつつ、最適化更新頻度を明示的に落とす。

## 3. ROS実装アーキテクチャ

### 3.1 ファイル構成（提案、実状に応じて調整）

```text
robots/dracomancer/
├── scripts/control/control_joint_angle.py  # 既存モード切替・安全ゲートは維持
├── scripts/control/shape_mapping.py         # 新規：ROS依存をできるだけ避けた幾何・最適化本体
├── config/control/shape.yaml                # 新規：形状点、軸合わせ、重み、最適化パラメータ
├── launch/teleoperation.launch              # mapping_mode:=shapeと設定読込のみ追加
└── docs/design-spec/05-software.md           # 実装内容と一致する場合のみ必要部分を更新
```

テストの置き場はリポジトリ規約とCatkin構成を確認したうえで決める。新しいROSトピックやメッセージ定義は、必要な場合に最小限とする。

### 3.2 制御経路

```text
/dracomancer/joint_states
       ↓ (nameで取り出し、stamp・finite・limits確認)
Human URDF FK → P_H → z_H
       ↓
shape_mapping: 最適化 → q_ref[6]
       ↓
既存 control_joint_angle.py の teleop_shape_target()
       ↓
既存 ShapeFeasibility サービス / 安全ゲート
       ↓
既存 rate_limit / flight-state gate / command publisher
       ↓
/dragon/joints_ctrl (sensor_msgs/JointState)
```

- `mapped_target()` に `shape` 分岐を追加する。**別ノードから `/dragon/joints_ctrl` に同時publishしない**。
- `make_joint_msg()` の現行実装は1周期で `mapped_target()` を複数回呼ぶ可能性がある。最適化結果は**入力時刻または制御周期ごとにキャッシュ**し、候補と診断が同じ値になるようにする。評価関数計算のたびに状態を進めない。
- 既存 `teleop_mode`、Hover条件、入力未準備時の抑止、`max_step`、ゲートのフェイルセーフを維持する。
- 既存Link4アンカー補償は現在 `distal` に条件付けされている。`shape` に自動で適用しない（機体移動へ意図せず接続するため）。
- `/dracomancer/shape_control_error` は現状**関節角候補と採用指令の差分6値**であり、形状空間のRMSEではない。意味を変えず、新規形状誤差は別トピックで公表する。
- `mapping_mode:=shape` 用に、`q_ref`、shape RMSE（無次元）、solver時間、反復数、成功/保持状態、可能ならヤコビアン特異値を診断出力する。既存のcandidateのJointStateトピックを再利用できるか先に確認する。

### 3.3 失敗時の状態遷移

入力欠損、JointStateの名前不足、NaN/Inf、古いタイムスタンプ、TF/URDF不整合、キャリブレーション未完了、最適化失敗の場合、**新しい形態を更新しない**。安全に既知の姿勢を維持できない状態は制御のenableを拒否し、既存の安全系へ状態を通知する。`shape` モードだけの障害が既存モードへ波及しないようにする。

## 4. 飛行制約の扱い：段階を分離する

**初期実装で主張できるのは「形状を最小二乗で転写し、既存の飛行可能性チェックに通した」ことまでです。** これは数学的な「安全形状多様体への最近点射影」と同じではありません。

### Phase A（必須）：マッピング単体

- `q_ref` を計算し、従来モードと同じ経路で `shape_feasibility/check_shape` に通す。
- 既存 `hold` ゲートを基準として使用し、候補が不適切なら最後の採用形態を維持する。サービス障害・`valid=false`・`stability_ok=false`・余裕不足で安易に通過させない。
- `soft_scale` は、縮小後の形態が再チェックされない場合があるため、**安全性を検証するまで新モードの推奨設定にしない**。
- `max_step` 後の実際の発行姿勢と評価に用いた候補は一致しない場合がある。安全性を厳密に主張するなら**発行する姿勢および必要な経路中間点もチェック**する必要があると明記する。
- 推力余裕評価は予測・準静的な判定であり、動的安定性や接触時の力学を保証しない。

### Phase B（Phase A合格後）：実現可能性を考慮した補正

- `q_ref` への接近を最優先にしつつ、既存サービスで**候補を検査したステップのみ**採用する方式（既存 `step_search` の安全面・頻度・タイムアウトを監査）を検討。
- 将来的に \(\min_{q\in\mathcal Q_{\mathrm{feasible}}} E_{\mathrm{shape}}(q)\) と定義できる制約付き最適化へ拡張できるが、**これを最初から完成済みと呼ばない**。
- サービスを関節ヤコビアン計算中に何十回も呼んで40Hzを阻害しないこと。最適化と飛行チェックは独立した更新頻度・時間予算を持つ設計にする。
- 既存安全閾値の数値はMk-I由来の可能性がある。実機適用の前に別途校正し、今回の実装で推測値へ置き換えない。

## 5. 検証・受入条件

### 5.1 運動学単体テスト

- [ ] Dracomancerの7関節名をシャッフルしたJointStateでも同じ形状が得られる。
- [ ] URDF FKから得た肩・肘・手首・手先が、既存TF/robot_state_publisherとの代表姿勢比較で一致する（許容誤差・試験姿勢を記録）。
- [ ] DRAGON簡易FKの `link4 → link1` の順序・軸・関節オフセットが実際のモデルと一致するか検証し、矛盾は修正案を提示する。
- [ ] 剛体並進および**固定装着姿勢の変化**には形状が不変で、肩関節の回転には反応する。
- [ ] 弧長正規化後、全長の異なる相似形状の形状誤差がほぼゼロになる。
- [ ] 純粋な回内回外や姿勢依存の情報欠落を検証し、入力形状ヤコビアンの数値ランク／特異値を代表姿勢で報告する。**7→6で全7DoFが可逆と主張しない**。

### 5.2 最適化単体テスト

- [ ] DRAGON FK自身から生成した到達可能な目標形状では、元の関節角と**同一でなくても**、形状RMSEが数値許容誤差内へ収束する（多解性を考慮）。
- [ ] 到達不可能な人体形状でも有限な解と誤差を返し、関節制限を破らない。
- [ ] 関節角の連続入力で不要な大ジャンプを発生させない。
- [ ] 入力NaN/Inf・関節欠損・仮想基準点未設定・発散時には保持し、不正指令をpublishしない。
- [ ] `K`、重み、正則化、初期姿勢の変更で解が妥当な方向へ変わる。
- [ ] 実環境の代表入力で40Hzループの処理時間分布（平均、95パーセンタイル、最大）を測定する。

### 5.3 ROS統合・回帰テスト

- [ ] `mapping_mode:=shape` でのみ新マッパーが動作し、デフォルトおよび `distal/joint_pairing/geometric` は以前と同じ出力を維持する。
- [ ] `q_ref`、実際に送信した `q_command`、形状誤差、flight gate判定がログで区別できる。
- [ ] Teleop無効、Hover外、通信断、feasibilityサービス失敗時に既存安全動作を維持する。
- [ ] `roslaunch dracomancer shape_task.launch dryrun:=true` と既存Gazebo環境で、他の `/dragon/joints_ctrl` publisherと競合しない。
- [ ] 既存 `shape_task` の `E_s`（DRAGONとDRAGON目標の関節空間由来の形状誤差）と、新しい**人体形状対DRAGON形状のRMSE**を別指標として記録する。
- [ ] 同一の入力軌跡・安全条件で固定対応マッピング（A）とShape-based（B）を比較できる。最初は **shape RMSE、コマンド連続性、計算時間、ゲート拒否率**を評価する。

### 5.4 実機安全

実機フライトは**本タスクの自動実行範囲外**。静止/シミュレーションでの検証後に、別途安全計画、最大関節速度、推力余裕、通信監視、緊急停止を確認すること。2018年のDRAGON論文で用いた0.17 rad/sは当該実験の条件であり、現在の実機へ無検証で一般化しない。

## 6. 作業順序と最終成果物

1. リポジトリと正本の調査結果を短く整理。座標系、関節順、仮想体幹点、現在のデフォルト・安全系を明文化する。
2. URDF FK・中心線・正規化・DRAGON FK照合を実装し、単体テストを作る。**仮想体幹点が未定の場合は動作を有効化しない設定で進める**。
3. 形状誤差と最適化器をROS非依存に実装して、到達可能/不可能形状と特異姿勢をテストする。
4. `control_joint_angle.py` に `shape` モードを追加し、既存ゲート、入力監視、制限機構に接続。明示的なlaunch引数でのみ起動可能にする。
5. `shape_task` のdry-run、Gazebo、既存モードの回帰テストを実施。新しい結果は既存ログの意味を変更せず追加する。
6. 必要なら `docs/design-spec/05-software.md` と `06-evaluation.md` の**該当節のみ**更新する。未決定仕様は `09-open-issues.md` に残し、勝手に確定させない。
7. 変更ファイル、数式との対応、ROSトピック、起動例、テスト結果、性能測定、制約、未解決事項、Catkin再ビルド要否を最終報告する。

**起動インターフェースの目標例**（既存launchへ引数を追加した後に使える形）：

```bash
roslaunch dracomancer teleoperation.launch mapping_mode:=shape
```

コンパイルやサービス定義変更が必要になった場合、無関係の全パッケージをビルドせず、影響範囲を説明したうえで最小のCatkinビルドを実行すること。

## 7. 文献上の位置づけ・主張の範囲

- Zhao et al., *Design, Modeling, and Control of an Aerial Robot DRAGON* (IEEE RA-L 2018)：DRAGONの6内部DoFと可変ロータ配置、飛行特性。
- Anzai et al., *Design, Modeling and Control of Fully Actuated 2D Transformable Aerial Robot with 1 DoF Thrust Vectorable Link Module* (IROS 2019)：飛行可能性・推力余裕を考慮した最適化の参考。**2D機体の式を4リンクDRAGONへ無条件に移植しない**。
- Kaneko et al., *A Teleoperation Framework for an Articulated Aerial Robot with Full DoF Mapping of Base Pose and Joint Angles*：関節角を対応させる既存遠隔操作研究との対比。
- OCRAなどの中心線ベースの異構造動作転写研究：発想の先行例。**「中心線を介すること自体」を新規性として主張しない**。

研究上の検証対象は、**「操作者の上肢形状を保持しつつ、異構造の多関節飛行ロボットに実行可能な目標形態を生成できるか」**とする。実装済み部分（形状マッピング／予測ゲート）と今後の研究課題（真の安全集合への射影、動的保証、接触作業、力覚提示）を常に区別すること。
