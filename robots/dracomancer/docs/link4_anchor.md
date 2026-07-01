# distal の link4 アンカー

> 状態: **実装済み・実験機能**（[../scripts/control/control_joint_angle.py](../scripts/control/control_joint_angle.py)
> `update_link4_anchor`）。関節操作時に DRAGON の **link4（腕先端側）の位置をワールドにおおよそ固定**
> するため、`joints_ctrl` と同じ目標関節角から link4 の相対位置を先に計算し、COG 位置を
> 同周期で補償する。既定は `link4_anchor_mode:=position_only` で、baselink 姿勢補償は行わない。
> `link4_anchor_mode:=full` では link4 姿勢まで補償するが、姿勢failsafeや高度低下に直結しやすいため
> 明示指定時のみ使う。機能全体は既定 ON（`enable_link4_anchor:=true`）。

## 背景・目的

distal は人間の腕関節を DRAGON 各関節へ絶対角で写像するが、`joints_ctrl` は**相対形状しか
決めない**。実飛行でどのリンクが世界に止まるかは「COG 位置保持 ＋ baselink(link2) 姿勢保持」で
決まり、既定では**運動学ルートの link1 側が止まって見える**（[../README.md](../README.md) 形状制御節 / 02_robot_model）。

操縦の直感としては、**腕先端側 link4 を基準**にし、手首を曲げれば link1 が振れ、肩を曲げれば
link1〜3 が振れる挙動が望ましい。この要求には link4 の6DoF完全固定までは不要なので、現在は
COG位置だけで link4 位置を固定する `position_only` を標準経路とする。link4姿勢まで完全固定しようとすると
baselink姿勢を大きく動かす必要があり、飛行安定性と衝突しやすい。

## 決定事項（実装に反映済み）

1. **distal の実験機能**として `control_joint_angle.py` 内に実装（別ノードにしない）。
2. **基準（link4 の固定目標姿勢）はホバー開始時にキャプチャ**する。
   - `flight_state == 5`（HOVER_STATE）への遷移時に、その時点の link4 world pose を記録。
   - `~recapture_anchor`(std_msgs/Empty) で再キャプチャ可能。ホバリングを抜けると基準は破棄し次のホバーで再取得。
3. **標準は位置だけ固定**。
   - `link4_anchor_mode:=position_only` では `uav/nav` の COG 位置だけを補償し、baselink RPY はpublishしない。
   - `link4_anchor_mode:=full` では旧来の6DoF寄り補償として COG位置 + baselink姿勢を補償する。
4. **危険なbody補償は送らない**。
   - `max_step` と body step scaling 後に、position_only では予測COG高度とホバー開始時COGからの水平距離、full ではさらにbaselink roll/pitchを検査する。
   - 安全域外なら `joints_ctrl` も body 補償も直前姿勢で保持する。
   - TFが取得できず補償目標を計算できない周期も、fail-safeとして直前姿勢を保持する。
   - `/target_rotation_motion` への即時姿勢指令は full モードでも既定OFF。

## DRAGON 制御の前提（確認済み）

- 位置制御点は **COG**（[../../../aerial_robot_control/src/flight_navigation.cpp](../../../aerial_robot_control/src/flight_navigation.cpp) は位置目標を COG として扱う）。
- `/<robot>/uav/nav` は DRAGON の `HOVER_STATE == 5` でのみ受理される。`LAND_STATE` や
  DRAGON独自の `PRE_LAND_STATE` 中は nav が無視され、link4アンカーは成立しない。
- position_only では姿勢は通常の DRAGON 側制御に任せ、Dracomancer から link4固定用の baselink RPY は送らない。
- full では **baselink = link2(`fc`)** 姿勢も補償する。さらに `enable_baselink_roll_mapping` が有効なら、
  上腕ロールと前腕ロールの中立値からの差分和を、逆算した baselink roll に加算する。
- full では互換・可視化用に `/<robot>/final_target_baselink_rpy`(Vector3Stamped, [r,p,y])
  をpublishする。さらに `/<robot>/target_rotation_motion`
  (nav_msgs/Odometry, `header.frame_id=baselink`) で絶対姿勢を即時指令する経路もあるが、
  姿勢failsafeへ近づきやすいため既定OFF（`publish_link4_anchor_baselink_motion:=false`）。
- `final_target_baselink_rpy` 単独では baselink 姿勢にスルーレート制限が入る
  （`baselink_rot_change_thresh`≈0.04 / `baselink_rot_pub_interval`≈0.2s → 約 0.2 rad/s）ため、
  関節変形に対するlink4固定補償が後れる。
- baselink を link4 に変えるのは不可（IMU/FC が物理的に link2 にある）。
- **DRAGON コア改造は不要**。Dracomancer 側で nav + joints を協調送信する。full モードのみ baselink_rpy も送る。

## 制御の考え方

link4 のワールド位置を基準 `T_anchor`（ホバー開始時キャプチャ）に保つよう、毎周期 COG 位置を
補償指令する。目標関節角 `q`（distal が算出）に対し、モデル/TF から `T_cog→link4(q)` を得て：

```text
COG_world.p = p_anchor - R_world→cog(current) · p_cog→link4(q) # position_only
joints_ctrl = q                                # 形状

# link4_anchor_mode:=full のみ:
COG_world      = T_anchor · T_cog→link4(q)^{-1}
baselink_world.M = T_anchor.M · R_baselink→link4(q)^{-1}
baselink_roll   += sum(upper_arm_roll_delta, forearm_roll_delta)
```

position_only では `joints_ctrl` と `uav/nav` を安全ゲート通過時だけ**同時送信**する。実装では `T_anchor` を TF（`world`→`<robot>/link4`）から取得し、
`T_baselink→link4(q)` は DRAGON v1/v1.5 の最小FK（`fc` 固定先の link2 から joint2/joint3 をたどる）で
`joints_ctrl` と同じ目標関節角から計算する。`T_cog→link4(q)` は現在TFの `cog→fc` と目標FKの
`fc→link4(q)` を合成する近似とする。position_only のCOG位置は現在TFの `world→cog` 姿勢を使って
「現在の姿勢のまま link4 位置だけを基準位置へ置く」ように解く。
形状は `max_step` で緩やかに変わるが、link4 補償は現在 link4 TF の後追いではなく、目標形状への
フィードフォワードとして行う。
`enable_link4_anchor_body_step_scaling` が有効な場合、`max_step` 適用後の候補姿勢が必要とする
COG位置目標の変化量を予測し、`link4_anchor_max_body_pos_rate` を超えると関節ステップ全体を縮小する。
full モードでは baselink姿勢目標の変化量も `link4_anchor_max_body_rpy_rate` で抑える。これにより、
形状変化の速さをbody補償が追従できる範囲へ合わせる。
さらに `enable_link4_anchor_body_safety` が有効な場合、縮小後の候補が必要とする
COG高度とホバー開始時COGからの水平距離を検査する。full モードでは baselink roll/pitch も絶対値で検査する。
`link4_anchor_min_cog_z` を下回る、`link4_anchor_max_cog_z` を超える、`link4_anchor_max_cog_xy_offset`
を超える、または full モードで `link4_anchor_max_abs_roll` / `link4_anchor_max_abs_pitch` を超える場合、その候補姿勢は採用せず
直前の関節姿勢を保持し、拒否されたbody補償はpublishしない。
同一制御周期内の step scaling / safety gate / nav publish は、周期先頭で更新した同じ TF を使う。
キャプチャ直後の初回補償は、現在TFの `cog→fc` と目標FKを使うため、通常は現在姿勢に近い指令から始まる。

```mermaid
flowchart TD
    H["hover 開始 (flight_state==5)"] --> CAP["link4 world pose をキャプチャ = T_anchor"]
    Q["distal: 目標関節角 q"] --> FK["TF/モデルで T_cog->link4(q)"]
    CAP --> COMP
    FK --> COMP["COG位置を逆算<br/>(fullのみbaselink姿勢も逆算)"]
    COMP --> SAFE{"COG z within safety bounds?<br/>(fullのみroll/pitchも検査)"}
    SAFE -->|yes| NAV["uav/nav (COG, POS_MODE)"]
    SAFE -->|fullのみ| RPY["final_target_baselink_rpy<br/>+ optional target_rotation_motion"]
    SAFE -->|yes| JC["joints_ctrl"]
    SAFE -->|no| HOLD["hold previous joint/body target"]
    Q --> SAFE
    NAV --> DRAGON
    RPY --> DRAGON
    JC --> DRAGON["DRAGON: COG位置を追従 → link4位置がほぼ固定"]
```

SVG版の全体図は [../figures/link4_anchor_algorithm.svg](../figures/link4_anchor_algorithm.svg) を参照。

## 注意点・限界（完全静止不要なので許容）

- position_only は link4 の姿勢までは固定しない。操縦直感を合わせるための「link4位置基準」の補償である。
- full モードは baselink姿勢のスルーレート制限で、速い関節操作時は link4 が一時的にズレ、遅れて追従する。
- full モードで補償姿勢が大きいとベクタリングのフィージビリティ限界や姿勢failsafeに当たる。既定では position_only とし、full は明示指定時だけ使う。
- 位置制御を使う必要がある。既存 `enable_position_control` の経路は `POS_VEL_MODE`＋`target_pos`
  未設定で落下するバグがあるため（[../README.md](../README.md) 既知の課題）、本機能は**絶対 COG 位置
  (POS_MODE)** を送る別ロジックで実装する。
- 操縦桿による移動（control_position）と同時利用すると、両者が `uav/nav` を書いて競合する。
  本機能 ON 時は移動制御（`enable_position_control`）を OFF にすること（移動制御は既定で OFF）。

## パラメータ（control_joint_angle）

| param | 既定 | 説明 |
| --- | --- | --- |
| `~enable_link4_anchor` | `true` | distal 時に link4 アンカーを有効化する |
| `~link4_anchor_mode` | `position_only` | `position_only`: link4位置だけをCOG位置で補償 / `full`: COG位置+baselink姿勢でlink4姿勢も補償 |
| `~anchor_link` | `link4` | 固定するリンク名（`<robot>/<anchor_link>`） |
| `~world_frame` | `world` | アンカー基準のワールドフレーム |
| `~cog_frame` / `~baselink_frame` | `<robot>/cog` / `<robot>/fc` | COG・baselink の TF フレーム |
| `~nav_topic` / `~baselink_rpy_topic` | `/<robot>/uav/nav` / `/<robot>/final_target_baselink_rpy` | 出力先 |
| `~baselink_motion_topic` / `~publish_baselink_motion` | `/<robot>/target_rotation_motion` / `false` | full モード用のbaselink即時姿勢指令。姿勢failsafeへ近づきやすいため既定OFF |
| `~enable_link4_anchor_body_step_scaling` | `true` | body目標の必要変化量に基づいて関節ステップを自動縮小 |
| `~link4_anchor_max_body_pos_rate` / `~link4_anchor_max_body_rpy_rate` | `0.4` / `0.8` | body step scalingで許容するCOG位置・baselink姿勢の最大変化速度。姿勢側はfullモードのみ使用 |
| `~enable_link4_anchor_body_safety` | `true` | body補償後のCOG高度を検査し、安全域外なら直前姿勢を保持。fullモードではbaselink姿勢も検査 |
| `~link4_anchor_max_abs_roll` / `~link4_anchor_max_abs_pitch` | `0.6` / `0.6` | fullモードのbody補償で許容するbaselink roll/pitch絶対値 [rad] |
| `~link4_anchor_min_cog_z` / `~link4_anchor_max_cog_z` | `0.6` / `2.5` | body補償で許容するCOG高度範囲 [m]。max `0.0` は上限無効 |
| `~link4_anchor_max_cog_xy_offset` | `1.0` | ホバー開始時COGから許容する水平距離 [m]。`0.0` 以下で無効 |
| `~enable_baselink_roll_mapping` | `true` | fullモード時に上腕ロール+前腕ロールの差分和をbaselink rollへ加算 |
| `~baselink_roll_source_joints` / `~baselink_roll_signs` / `~baselink_roll_scales` | `[upper_arm_external_internal_rotation_joint, wrist_supination_joint]` / `[-1,-1]` / `[1,1]` | baselink roll 差分の入力・符号・ゲイン |
| `~baselink_roll_limit` | `pi/2` | baselink roll へ加算する差分の絶対値上限 [rad] |
| `~dragon_link_length` / `~dragon_inter_joint_x_offset` / `~dragon_link2_fc_xyz` | `0.474` / `0.02575` / `[0.3245, -0.0010, 0.0280]` | link4 アンカー用の最小 DRAGON FK パラメータ |
| `~hover_flight_state` | `5` | DRAGON の HOVER_STATE。link4アンカー補償を出す flight_state |
| `~recapture_anchor`(Empty) | – | 基準の再キャプチャ要求 |

## 残課題 / TODO

- 実機（Gazebo）での再確認（ズレ量、追従、安全ゲートの保持動作）。
- z（高度）・link4位置ズレ量の追従品質確認。
- 移動制御（操縦桿）との両立（uav/nav の合成）。
- 必要なら COG 変化も含むモデルFK（aerial_robot_model）に切替えて、位置補償の精度を上げる。
