# falling_test_20260705_222212.bag analysis

対象bag: `~/rosbag/falling_test_20260705_222212.bag`

## 結論

- 墜落トリガは高度低下ではなく姿勢failsafe。
  - `rt=211.742 s`: `failsafe: the roll pitch angles are too large, roll: -1.002745 (1.000000), pitch: 0.027956 (1.000000)`
  - `rt=211.756 s`: `/dragon/flight_state` が `5` から `17` へ遷移。
  - `rt=213.944 s`: ホバー後初めて `z < 0.3 m`。
- feasibility gateは候補姿勢の予測fcだけを見るため、実測fc低下・姿勢ロール増大を止められていない。
  - 候補fcは墜落まで全点でhardしきい値以上。
  - 実測filtered fcは墜落までにforceで45点、torqueで690点がhardしきい値未満。
  - failsafe直前の `/dracomancer/dragon_shape_safety_scale` は `0.0` だが、これは情報提供用でgateには使われない。

## 主な数値

| 項目 | 値 |
| --- | --- |
| force hard threshold | `0.10899` |
| torque hard threshold | `0.01540` |
| candidate force min | `2.01316` |
| candidate torque min | `0.09555` |
| actual filtered force min | `0.01679` |
| actual filtered torque min | `0.0` |
| failsafe直前 roll | `-0.99063 rad` |
| failsafe直前 pitch | `0.02842 rad` |
| failsafe直前 safety scale | `0.0` |

failsafe直前の指令関節角:

| joint | command [rad] | candidate [rad] |
| --- | ---: | ---: |
| joint1_pitch | `-0.06443` | `-0.06443` |
| joint1_yaw | `-0.01074` | `-0.01074` |
| joint2_pitch | `0.0` | `0.0` |
| joint2_yaw | `-0.14419` | `-0.14419` |
| joint3_pitch | `0.27458` | `0.27305` |
| joint3_yaw | `0.00767` | `0.00767` |

## gateが効いていない要因

`control_joint_angle.py` のfeasibility gateは、`/dragon/shape_feasibility/check_shape` に候補関節角を渡し、予測 `fc_f_min` / `fc_t_min` がhardしきい値以上かだけで採否を決める。今回のbagでは、候補fcは最後まで安全判定で、`deform=False` 相当の保持には入っていない。

一方で、実際のDRAGON側のfiltered fcは頻繁にhardしきい値を下回り、failsafe直前にも `safety_scale=0.0` になっている。つまり原因は「候補姿勢の予測fcが危険姿勢を返したのにgateが通した」ではなく、以下の不一致。

- 予測fcと実測fcの乖離。
- gateが実測fc・filtered fc・safety scaleを採用可否へ戻していない。
- `link4_anchor_mode=position_only` ではCOG位置だけを出し、baselink roll/pitch安全判定は行わない。
- ロールが `1.0 rad` のfailsafe閾値へ急増する動的状態は、静的な候補形状fc gateでは直接評価されない。

## 改善案

1. 実測fc/safety scaleを形状gateへフィードバックする。
   - `/dracomancer/dragon_shape_safety_scale == 0` または `/dragon/debug/fc_*_min_filtered < hard_min` の間は `last_feasible_target` を保持する。
   - 解除は既存のrecoverしきい値相当でヒステリシスを入れる。

2. 姿勢failsafe予兆をshape/link4側で止める。
   - `/dragon/uav/baselink/odom` または `/dragon/ground_truth` のroll/pitchが例えば `0.6 rad` を超えたら形状更新とlink4 anchor nav更新を止める。
   - `position_only` でもroll/pitch監視を有効にする。

3. 予測fcと実測fcの差分を記録・可視化する。
   - 今回のbagではcandidate torque minは最低 `0.09555` だが、actual torque minは `0.0`。
   - `shape_feasibility_node` の `optimized_gimbal` 予測と実際のgimbal/controller状態の差を、bagから定量評価する必要がある。

4. link4 anchorのCOG目標を姿勢安定性で制限する。
   - 現状のbody step scalingは位置目標の変化量を抑えるが、roll/pitch悪化そのものは止めない。
   - COG targetの横移動・高度変化に加えて、実機姿勢応答を見たrate制限を追加する。

## 実装済みの予測改善

この解析後、予測fcを実測fcへ近づけるために以下を実装した。

- `shape_feasibility_prediction_mode=controller` を既定化。
  - 最新 `/dragon/gimbals_ctrl` のgimbal roll指令を使い、controller側のgimbal条件へ寄せる。
  - `final_target_baselink_rpy` または任意で `uav/baselink/odom` の姿勢feedbackを使い、torque fc計算の姿勢条件へ反映する。
  - gimbal feedbackが新鮮でない場合は従来の `optimized_gimbal` へfallbackする。
- `shape_feasibility_prediction_mode=allocation` を既定化。
  - 候補形状に対して静的ホバーallocationを行い、割当後のgimbal roll角でfcを再評価する。
  - `controller` modeは最新gimbal指令を使う旧近似として残す。
- gate前candidateだけでなく、rate limit / link4 safety後に実際へ送る最終targetの予測fcを追加。
  - `/dracomancer/target/fc_f_min`
  - `/dracomancer/target/fc_t_min`

残る課題は、DRAGON controller内部のallocation関数そのものを共有ライブラリ化し、`shape_feasibility_node` から完全に同じ割当器を呼ぶこと。現状の `allocation` mode はDracomancer側に必要最小限の静的allocationを再実装している。

## 出力

- 関節対応グラフ: `robots/dracomancer/figures/falling_test/joint_mapping.pdf`
- 1関節1PNG/SVG: `robots/dracomancer/figures/falling_test/joint*.png`, `robots/dracomancer/figures/falling_test/joint*.svg`
- 安全概要: `robots/dracomancer/figures/falling_test/safety_overview.png`, `robots/dracomancer/figures/falling_test/safety_overview.svg`
- 予測fcと実測fcの比較・差分: `robots/dracomancer/figures/falling_test/fc_prediction_error.png`, `robots/dracomancer/figures/falling_test/fc_prediction_error.svg`
- 数値要約: `robots/dracomancer/figures/falling_test/summary.txt`
- 生成スクリプト: `robots/dracomancer/figures/plot_falling_test.py`

この環境はDISPLAYなしのため、`matplotlib` の画面表示は未実行。表示が必要な場合はGUI環境で以下を実行する。

```bash
python3 robots/dracomancer/figures/plot_falling_test.py ~/rosbag/falling_test_20260705_222212.bag --show
```

Catkin再ビルドは不要。
