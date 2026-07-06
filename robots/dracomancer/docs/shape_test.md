# shape-test_20260706_175417.bag analysis

対象bag: `/home/oku/rosbag/shape-test_20260706_175417.bag`

生成物:

- `robots/dracomancer/figures/shape_test_20260706_175417/summary.txt`
- `robots/dracomancer/figures/shape_test_20260706_175417/safety_overview.{png,svg}`
- `robots/dracomancer/figures/shape_test_20260706_175417/fc_prediction_error.{png,svg}`
- `robots/dracomancer/figures/shape_test_20260706_175417/joint_mapping.pdf`
- `robots/dracomancer/figures/shape_test_20260706_175417/joint*.{png,svg}`

## 結論

- 直接の墜落トリガは姿勢failsafe。
  - `rt=83.566 s`: `failsafe: the roll pitch angles are too large, roll: -1.003392 (1.000000), pitch: 0.156559 (1.000000)`
  - 同時刻に `FORCE LANDING MSG From AERIAL ROBOT`。
  - `rt=83.571 s`: `/dragon/flight_state` が `5` から `17` へ遷移。
- 高度低下もほぼ同時に発生している。
  - `rt=83.000 s`: baselink `z=0.884 m`, `roll=-0.045 rad`。
  - `rt=83.534 s`: baselink `z<0.3 m`。
  - `rt=83.566 s`: ground truth `z=0.240 m`, `roll=-1.003 rad`。
- `/dragon/uav/nav` は墜落直前も低高度を指令していない。
  - `rt=83.534 s`: COG target `z=0.945 m`。
  - `rt=83.566 s`: COG target `z=1.002 m`。
  - よって原因は「低いZ目標を送ったこと」ではなく、姿勢・高度制御の過渡破綻。
- shape task自体は `state=idle` のままで、試行開始・成功・失敗イベントは記録されていない。

## 時系列

| rt [s] | 事象 |
| ---: | --- |
| 0.000 | `/dragon/flight_state=5` |
| 50.147 | `/dracomancer/shape_task/status` は `idle`、trial 0、targetなし |
| 82.190 | `shape_safety state=safe`, measured torque radius `0.3315` |
| 82.309 | `shape feasibility: deform=True`, predicted `fc_f_min=1.4300`, `fc_t_min=0.0290` |
| 83.000 | baselink `z=0.884 m`, `roll=-0.045 rad`; live safety scaleは直近で `0.0` |
| 83.212 | `shape_safety state=warning`, measured torque radius `0.0387`, safety scale `0.089` |
| 83.365 | `shape feasibility: deform=True`, predicted `fc_f_min=0.8717`, `fc_t_min=0.1213` |
| 83.491 | `abs(roll/pitch)>0.4` |
| 83.512 | `abs(roll/pitch)>0.6` |
| 83.534 | baselink `z<0.3 m` |
| 83.545 | `abs(roll/pitch)>0.8` |
| 83.566 | 姿勢failsafe、force landing、ground truth `roll=-1.003 rad` |
| 83.571 | `/dragon/flight_state=17` |

## 安全余裕

しきい値:

| channel | hard | recover |
| --- | ---: | ---: |
| force | 0.10899 | 0.24922 |
| torque | 0.01540 | 0.27816 |

墜落までの最小値:

| signal | min | hard未満 |
| --- | ---: | ---: |
| candidate force fc | 0.8276 | 0 / 951 |
| candidate torque fc | 0.0282 | 0 / 951 |
| target force fc | 0.8720 | 0 / 525 |
| target torque fc | 0.0261 | 0 / 525 |
| measured force fc filtered | 0.0501 | 44 / 2448 |
| measured torque fc filtered | 0.0 | 154 / 2398 |

候補・最終targetの予測fcはhardしきい値未満にならないため、feasibility gateは最後まで保持に入らず `deform=True` 相当で通している。一方で実測filtered fcはhard未満を複数回記録し、torqueのrecover未満も多い。墜落直前の実測torque fcは `0.0440` でhardより上だがrecoverより下、safety scaleは `0.109` まで低下していた。

## 指令と追従

墜落直前の `/dragon/uav/nav`:

| rt [s] | x | y | z |
| ---: | ---: | ---: | ---: |
| 83.000 | 0.0388 | -0.0568 | 0.9079 |
| 83.534 | -0.1069 | -0.0427 | 0.9452 |
| 83.566 | -0.1621 | -0.0311 | 1.0017 |

墜落直前の `/dragon/joints_ctrl`:

| joint | command [rad] | actual [rad] |
| --- | ---: | ---: |
| joint1_pitch | -0.1580 | -0.1970 |
| joint1_yaw | -0.3728 | -0.2693 |
| joint2_pitch | -0.4939 | -0.5586 |
| joint2_yaw | 0.0000 | 0.0140 |
| joint3_pitch | 0.7424 | 0.6605 |
| joint3_yaw | 0.1703 | 0.2207 |

墜落直前の関節追従誤差はおおむね `0.01--0.10 rad` 程度で、単独の主原因とは言いにくい。ただし83秒台ではgimbal指令が大きく振れている。

墜落直前の `/dragon/gimbals_ctrl`:

| gimbal | roll [rad] | pitch [rad] |
| --- | ---: | ---: |
| 1 | 0.816 | -0.398 |
| 2 | 1.380 | -0.234 |
| 3 | 0.099 | 0.284 |
| 4 | 0.620 | -0.436 |

`rt=82--83.566 s` の範囲では `gimbal4_roll` が `-2.556--2.879 rad`、`gimbal2_pitch` が `-2.559 rad` まで振れている。姿勢崩壊の直前に、ベクトル推力配分・姿勢応答が大きく過渡化している。

## 原因推定

実測から断言できる直接原因は、`roll` がfailsafeしきい値 `1.0 rad` を超えたこと。高度は同じ0.05秒程度の窓で急落しており、地面近傍化と姿勢崩壊は強く結合している。

より上流の要因としては、次が濃厚。

- link4 anchor由来のCOG位置目標と形状指令の過渡に対して、DRAGON側の姿勢・高度制御が追従できなかった。
- hardしきい値だけを見る予測feasibility gateでは、recover未満やlive safety scale低下を止められなかった。
- 実測torque余裕とgimbal状態が不安定な状態でも、候補姿勢の予測fcはhard以上だったため、形状更新が継続した。

## 改善案

1. 実測安全余裕を形状gateに戻す。
   - `/dracomancer/dragon_shape_safety_scale` が低い、または `/dragon/debug/fc_*_min_filtered` がhard/recover未満の間は形状更新とlink4 anchor nav更新を一時保持する。

2. `position_only` link4 anchorでも姿勢・高度監視を入れる。
   - baselink/cog `z` が低い、または `abs(roll/pitch)` が `0.4--0.6 rad` を超えたら、joint target更新とCOG target更新を止める。

3. recoverしきい値をgateに使うヒステリシスを強める。
   - 今回はtarget torque予測がrecover未満のサンプルが多く、hardだけでは余裕不足を見逃す。

4. gimbal過渡を安全条件に入れる。
   - gimbal roll/pitchが大きく振れた周期は、静的fc予測よりも実機応答を優先して保守的に保持する。

Catkin再ビルドは不要。
