# control_joint_angle.py 詳細レビュー（関節マッピングの直感性問題）

- 日付: 2026-07-07
- 対象: `scripts/control/control_joint_angle.py`（`mapping_mode=distal` 既定構成、`launch/teleoperation.launch` の既定引数前提）
- 背景: Dracomancer の関節角変化に対して DRAGON が「思ったように（直感的に）動かない」という症状の原因調査。
- 手法: コード精読 + ロール配分ロジックの数値検証（実機・sim 未実施）。

## 結論（重大度順）

| # | 重大度 | 指摘 | 症状への寄与 |
| --- | --- | --- | --- |
| 1 | **重大** | 肘ロール配分が上腕ロール基準角(r=0)で**不連続**。`joint2_yaw` 目標が ±flexion（肘90°曲げで計180°）跳ぶ | 肘を曲げたまま上腕ロールが 0 をまたぐたび DRAGON joint2 が反対側へ大旋回。「符号が逆に見える」症状の正体の可能性大 |
| 2 | **重大** | ロール配分の基準が**絶対角0固定**（中立キャプチャなし）。操縦者の自然姿勢がちょうど #1 の不連続境界に乗る | まっすぐ曲げたつもりでも pitch/yaw が混ざる。姿勢による再現性のない挙動 |
| 3 | **重大** | teleoperation 開始時の**大ジャンプ**。distal は `mapping_reference=circular` を無視しており、円形離陸形状を腕で保持することは可動域的にほぼ不可能 | モード切替直後に意図しない大変形。feasibility gate が噛むと途中で全関節フリーズ |
| 4 | 中 | link4 アンカー系の**全関節ホールド経路**が多数（TF未取得は無言、追従誤差・body安全・関節追従で hold） | 「動かない/急に止まる」。ログでの切り分けが必要 |
| 5 | 中 | hold gate は**全関節一括**判定、かつ torque の recover しきい値が hard の約18倍で解除が遠い | 一度 hold に入ると腕を大きく戻すまで全関節フリーズ |
| 6 | 中 | 同期 TF lookup（timeout 0.1s×3/周期）+ 同期サービス呼び出しでループ周期が低下し得る。`max_step` は周期毎なので実効速度も低下 | 全体に「遅い・鈍い」体感 |
| 7 | 軽微 | switching ON 時 `distal_signs/scales/offsets` の joint1/joint2 成分が**死にパラメータ**（switching 側の別パラメータで上書き） | 符号チューニングが効かない罠。直近の符号反転作業の混乱要因 |
| 8 | 軽微 | 手首回転配分は `wrist_pitch_sign*scale == wrist_yaw_sign*scale` のときだけ正しい回転。片側だけ変更すると歪む | チューニング時の罠 |
| 9 | 軽微 | `convert_servo_to_joint_states.py` がサーボ欠測を **0.0 埋め**するため、絶対一致写像が「0姿勢への指令」と解釈する | サーボ断で DRAGON が勝手に動く（別パッケージ、提案のみ） |
| 10 | 軽微 | ドキュメント不整合（AGENTS.md の `distal_signs` 旧値、`mapping_reference` は「全モード共通」とコメントされるが distal では未使用） | — |

## 1. 肘ロール配分の不連続（本命）

[control_joint_angle.py:789-798](../scripts/control/control_joint_angle.py#L789-L798):

```python
signed_theta = roll_components[0]           # 飽和済み上腕ロール差分 r
theta = abs(signed_theta)
pitch_weight = theta / (np.pi / 2.0)        # 偶関数: |r| に比例
yaw_direction = -1.0 if signed_theta < 0.0 else 1.0
yaw_weight = yaw_direction * (1.0 - pitch_weight)  # r=0 で ±1 に跳ぶ
```

数値検証（flexion=1.2 rad、既定符号）:

| 上腕ロール r [rad] | joint2_pitch | joint2_yaw |
| --- | --- | --- |
| -0.010 | +0.008 | **+1.192** |
| 0.000 | 0.000 | **-1.200** |
| +0.010 | +0.008 | **-1.192** |
| ±1.571 | **+1.200（同符号）** | 0.000 |

問題点は2つで、どちらも「偶関数/奇関数の割り当てが手首側と逆」に帰着する。

1. **不連続**: `yaw_weight` が r=0 で ±(1-ε) 間を跳ぶ。肘を曲げた状態でロールが基準をまたぐと `joint2_yaw` 目標が 2×flexion 反転し、`max_step`（0.6 rad/s）で数秒かけて反対側へ旋回する。基準は `elbow_roll_offset=0`（絶対角）なので、**組立零点付近の自然姿勢がまさに境界**。
2. **pitch がロール方向で反転しない**: r=±90° のどちらでも pitch は同符号。物理的には曲げ面がロールとともに回転するので、ロール方向で pitch の向きは反転すべき（手首側の cos/sin 実装はそうなっている）。

この2点は commit `bf3ccc0c`（joint2_yaw 符号反転）の背景と整合的: **腕がどちら側のロールに居たかで符号が逆に見える**ため、符号反転では解決せず境界の反対側へ症状が移るだけ。

### 修正案A（推奨・最小差分）※2026-07-07 実装済み

符号（奇関数）を pitch 側へ、偶関数を yaw 側へ入れ替える:

```python
pitch_weight = signed_theta / (np.pi / 2.0)          # 奇: ロール方向が pitch の向きを決める
yaw_weight = 1.0 - abs(signed_theta) / (np.pi / 2.0) # 偶: 中立で yaw 最大、連続
```

- r=0 近傍: `joint2_yaw = elbow_yaw_sign*flexion` で安定（連続）。
- r=±90°: `joint2_pitch = ±flexion`（ロール方向に応じ反転、手首と整合）。
- 三角関数版 `pitch_weight=sin(r)`, `yaw_weight=cos(r)` にすれば手首（cos/sin 回転）と完全に同型になる。L1線形（現行 README 記載の theta:(π/2-θ) 比）を保つなら上記の線形版。
- 注意: この修正で `bf3ccc0c` の `elbow_yaw_sign=-1` 反転が正しかったかは再検証が必要（症状対処だった可能性）。sim で肘曲げ→ロール掃引を再確認すること。
  → 2026-07-07 指示により `elbow_yaw_sign` と `distal_signs[2]` を bf3ccc0c 以前の `+1` へ戻した（sim での向き確認は未実施）。
- `rho`（switch_ratio 診断）は `abs(yaw_weight)` 定義のままで整合。README の「上腕ロールの符号はjoint2_yaw側に反映」記述の更新が必要。

## 2. ロール基準の絶対角0固定

- `elbow_roll_offset` / `wrist_roll_offsets` は既定 0、かつ distal は中立キャプチャをしない（[control_joint_angle.py:102-113](../scripts/control/control_joint_angle.py#L102-L113)）。
- サーボ読みは tick 2048 中心の絶対角（`convert_servo_to_joint_states.py`）なので、**操縦者の自然姿勢のロール読みが 0 である保証がない**。ずれていると手首・肘の pitch/yaw 配分が常に事前回転され、「屈曲だけ動かしたのに斜めに曲がる」。
- baselink roll 側は中立キャプチャ方式（ホバー開始時等）を採用しており、**同じロール関節に対して基準の思想が二重**。

### 修正案C ※2026-07-07 `capture_roll_neutral` として実装済み

ホバー開始 / teleoperation 切替 / `~recapture_anchor` 受信時に、現在のロール値を `elbow_roll_offset` / `wrist_roll_offsets` としてキャプチャするオプション（例: `~capture_roll_reference_on_engage`、既定 ON 推奨）を追加する。既存の `want_capture_baselink_roll_neutral` トリガに相乗りでき、変更は小さい。修正案Aとセットで「自然姿勢＝連続な yaw 純点」になる。
→ 一旦、全ソース関節＋基準形状をキャプチャする `distal_relative` として案B1と統合実装したが、指示により**配分ロール（上腕・前腕）のみ**のキャプチャ（`capture_roll_neutral` 既定 true）へ変更。屈曲・内外転系は絶対角一致を維持する。

## 3. teleoperation 開始時のジャンプ（mapping_reference 未適用）

- `mapping_reference`（launch 既定 `circular`）はコメント上「全 mapping_mode 共通」だが、実際は **joint_pairing の既定オフセットにしか使われていない**（[control_joint_angle.py:58-71](../scripts/control/control_joint_angle.py#L58-L71), [95-100](../scripts/control/control_joint_angle.py#L95-L100)）。distal は `distal_offsets=[0,...]` の絶対一致。
- startup 円形 `[0, π/2, 0, π/2, 0, π/2]` を distal 写像で保持するには、例えば `joint1_yaw=+π/2` に手首内外転 −90°、`joint3_yaw=+π/2` に肩内外転 −90° が必要で、**人体可動域的にほぼ不可能**。つまり teleop へ切り替えた瞬間、候補は必ず円形から大きく離れ（概ね直線形状方向）、DRAGON が数秒かけて意図しない大変形をする。直線寄り形状で fc が下限を割れば gate hold で途中フリーズ。

### 修正案B（いずれか） ※2026-07-07 B1として一旦実装後、指示により撤回（屈曲・内外転系は絶対角一致を維持。teleop開始時のジャンプはサーボ零点較正・運用で対処）

- **B1（コード・推奨）**: distal にも `mapping_reference` を実装する。teleoperation 切替（またはホバー開始）時の腕角 `source0` をキャプチャし、`target = ref + sign*scale*(source - source0)`（`ref` は `circular` なら `startup_pose`、`straight` なら 0）とする相対変形。切替がバンプレスになり、円形を保ったまま「そこからの変形」を腕で指示できる。
- **B2（設定のみ・応急）**: 既存パラメータで「脱力姿勢＝円形」に寄せる: `wrist_yaw_offset:=1.5708`、`elbow_yaw_offset:=1.5708`、`distal_offsets:=[0,0,0,0,1.5708]`（joint3_yaw）。ただし絶対一致のままなので可動域の片側性が残る（肘屈曲は片方向のみ→`elbow_yaw_scale:=2.0` で ±90° をカバーする等の調整が必要）。

## 4. link4 アンカー系のホールド経路（診断ガイド）

「動かない」ときにどの経路で止まっているかをログで切り分ける:

| 経路 | 条件 | ログ |
| --- | --- | --- |
| TF 未取得 | `anchor_mat`/`cog_fc_mat` None | `link4 anchor TF refresh failed`（2s throttle）。ただし [control_joint_angle.py:1105-1106](../scripts/control/control_joint_angle.py#L1105-L1106) の hold は**無言** |
| body 安全 | 高度/水平リーシュ/（full時）姿勢 | `link4 anchor body safety rejected target: ...` |
| 追従安全 | COG/yaw/roll/pitch 追従誤差 | `link4 anchor tracking safety holds target: ...` |
| 関節追従 | 追従誤差 > 0.3 rad | `link4 anchor joint tracking safety holds target: ...` |
| feasibility hold | fc 予測が hard 未満 | `shape feasibility: deform=False ... holding=True` |

提案:

- 無言 hold 経路（step scaling の `anchor_mat is None` 分岐）に `logwarn_throttle` を追加。
- hold 系は全関節一括停止のため、体感上は「全部固まる」。原因表示（上記ログ）を web UI / RViz テキストへ出すと運用が楽になる。

## 5. hold gate の解除条件

- launch 既定: hard(force/torque)=0.1090/0.0154、recover=0.2492/0.2782。**torque は hard の約18倍まで回復しないと解除されない**。
- 較正データ由来だが、「一度止まると腕を大きく戻すまで動かない」の主要因になり得る。`feasibility_gate_mode:=step_search`（実装済み）を試すと、フリーズではなく「行ける所まで進む」挙動になり体感が改善する可能性がある。

## 6. ループ周期の低下要因

- `prepare_link4_anchor_tf()` は毎周期最大3回の `lookup_transform(timeout=0.1s)`、gate/target-fc は同期サービス呼び出し（allocation 最適化）。TF やサービスが遅いと 40 Hz ループが落ち、`max_step`（周期毎 0.015 rad）による実効関節速度も比例して低下する。
- 提案: 実機で `/dragon/joints_ctrl` の実 publish レートを確認（`rostopic hz`）。低ければ lookup timeout の短縮（例 0.03s）や、`rate_limit` を実測 dt ベース（rad/s 指定）へ変更する。

## 7-10. 整備事項

- **7**: `enable_*_roll_switching` ON 時、`distal_signs[0..2]`/`distal_scales[0..2]` 等は上書きされ無効。起動ログの `distal absolute match:` 行が有効に見えるのも誤解の元。switching ON 時は該当エントリを起動ログから除外 or「(overridden by roll switching)」を付記する。
- **8**: 手首配分が回転であるためには `wrist_pitch_sign*wrist_pitch_scale == wrist_yaw_sign*wrist_yaw_scale` が必要。差がある場合に logwarn を出す。ロール系全体の向き反転用に `wrist_roll_signs` / `elbow_roll_sign`（飽和前の差分に乗算）を追加すると、ハード側の回転方向不一致を綺麗に補正できる（現状は表現不能）。
- **9**: converter の 0.0 埋めは絶対一致写像と相性が悪い（欠測＝0姿勢指令）。converter 側で未受信 ID を name/position から除外する修正を提案（パッケージ内・要相談）。
- **10**: ドキュメント修正: AGENTS.md の `distal_signs 既定 [1,-1,1,1,-1]` は旧値（現行 `[1,-1,-1,1,-1]`）。`mapping_reference` の「Shared by every mapping_mode」コメント（コード内）と README の distal 節も、実装（distal では未使用）か修正案B実装後の仕様に合わせる。

## 実施推奨順

1. 修正案A（肘配分の連続化・**実装済み**）→ sim で肘曲げ+ロール掃引を確認（`/dracomancer/joint_map/switch_ratio` と `/dracomancer/candidate/joint_target` を記録）
2. 修正案C（ロール基準のエンゲージ時キャプチャ・**`capture_roll_neutral` として実装済み**）
3. 修正案B1（distal の相対変形対応・**実装後に指示で撤回、絶対角一致を維持**）
4. 診断ログ追加（#4）・ドキュメント整合（#10）
5. 実機で #5/#6 の実測（gate ログ・ループ周期）

## 確認したこと / 未確認なこと

- 確認: コード読解、ロール配分の数値検証（Python で再現）、launch 既定値、直近コミット履歴との整合。
- 未確認: 実機/sim での再現（特に修正案Aの符号が実機の直感と一致するか）、`shape_feasibility` サービスの応答時間、実ループ周期、TF 構成。
