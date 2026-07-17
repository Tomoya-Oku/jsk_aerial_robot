# eeb9efe test non-fc factors

解析対象:

- `/home/oku/rosbags/commit_eeb9efe08666091af6b26470bbb533ee9e7b22bd_test_20260705_235013.bag`
- 主な対象: `/dragon/uav/nav`, `/dragon/ground_truth`, `/dragon/uav/cog/odom`, `/dragon/joints_ctrl`, `/dragon/joint_states`, `/dragon/gimbals_ctrl`, `/dragon/debug/pose/pid`, `/dragon/four_axes/command`, `/dragon/motor_pwms`, `/dragon/estimated_external_wrench`

生成物:

- `robots/dracomancer/figures/crash_factors.py`
- `robots/dracomancer/figures/eeb9_factors/factors_overview.{png,svg}`
- `robots/dracomancer/figures/eeb9_factors/event_window.{png,svg}`
- `robots/dracomancer/figures/eeb9_factors/joint_velocity.{png,svg}`
- `robots/dracomancer/figures/eeb9_factors/summary.txt`

## 結論

fc以外で最も強い墜落要因は、DRAGONの関節角速度そのものより、link4 anchor由来の `/dragon/uav/nav` 目標COG高度が大きく下がったこと。

- bag開始後 `94.333 s` から `101.634 s` にかけて、`target_pos_z` が約 `1.02 m` から最小 `0.614 m` まで低下。
- 実高度はその後を追って低下し、`96.992 s` に `ground_truth z < 0.3 m`、`97.296 s` に最小 `0.288 m` になった。
- 同時刻のCOG高度は `0.459 m`、目標zは `0.637 m` であり、完全な自由落下ではなく「低い目標高度を追従し、さらに下方へオーバーシュート」した形。
- `/dragon/flight_state` は全区間 `5` で、明示的なfailsafe/landing遷移はない。

## 目標高度と沈下

`target_pos_z < 0.8 m` になった区間:

| start [s] | end [s] | target z min time [s] | target z min [m] | actual z at target min [m] |
| ---: | ---: | ---: | ---: | ---: |
| 43.821 | 49.130 | 45.151 | 0.601 | 0.647 |
| 78.194 | 84.071 | 79.092 | 0.711 | 0.678 |
| 94.333 | 101.634 | 96.215 | 0.614 | 0.440 |
| 101.746 | 102.900 | 101.972 | 0.741 | 0.655 |

最悪の沈下は3番目の区間。`target_pos_z` の低下が実高度低下より先行しているため、主因は推力不足や姿勢破綻ではなく、link4 anchor補償がCOG目標を下げすぎたことと考えるのが妥当。

## 関節角速度

イベント窓 `92.992..100.992 s` の主な値:

| joint | actual max abs velocity [rad/s] | command max abs rate [rad/s] |
| --- | ---: | ---: |
| joint1_pitch | 2.043 | 1.382 |
| joint1_yaw | 0.660 | 1.355 |
| joint2_pitch | 1.303 | 2.587 |
| joint2_yaw | 4.342 | 1.883 |
| joint3_pitch | 0.433 | 0.920 |
| joint3_yaw | 1.087 | 0.669 |

`nav_z_rate` と関節指令速度の相関（イベント窓）:

| joint | corr(nav_z_rate, command_rate) | max abs command rate while nav_z descends [rad/s] |
| --- | ---: | ---: |
| joint1_pitch | -0.036 | 1.230 |
| joint1_yaw | 0.194 | 1.094 |
| joint2_pitch | 0.678 | 1.781 |
| joint2_yaw | -0.584 | 1.781 |
| joint3_pitch | 0.039 | 0.213 |
| joint3_yaw | -0.327 | 0.141 |

`joint2_pitch` / `joint2_yaw` の変化が `nav_z` 低下と強く同期している。これは肘周りの形状変化に対してlink4固定補償がCOGを大きく下げていることを示す。

## 関節追従遅れ

イベント窓での指令-実角の最大誤差:

| joint | max abs error [rad] | time [s] | RMSE [rad] |
| --- | ---: | ---: | ---: |
| joint1_pitch | 0.224 | 93.813 | 0.063 |
| joint1_yaw | 0.792 | 92.993 | 0.239 |
| joint2_pitch | 0.493 | 100.974 | 0.137 |
| joint2_yaw | 0.409 | 95.495 | 0.163 |
| joint3_pitch | 0.053 | 100.173 | 0.026 |
| joint3_yaw | 0.033 | 100.515 | 0.008 |

追従遅れは存在するが、墜落直前に目標zが先に低下しているため、追従遅れだけが主因ではない。低高度目標に対して、関節・gimbalの過渡遅れとfc低下が重なり、下方オーバーシュートを増幅したと見るべき。

## 姿勢・推力・外乱

イベント窓:

- roll/pitch は最大でも約 `0.096 / 0.111 rad` 程度で、姿勢破綻が主因とは見えない。
- `pid_z_total` は `10.224` まで上がり、`thrust_sum` も `73.07` まで上がっている。推力指令が抜けた形ではない。
- イベント時のPWMは `1425..1558` で、明確な飽和ではない。
- 外力推定は `96.195 s` に `4.21 N` のピークがある。沈下を悪化させた可能性はあるが、より大きな外力ピークが別時刻にもあり、単独主因とは判断しにくい。
- gimbal pitch系はイベント窓で `13..14 rad/s` 程度の高速応答をしている。これはallocation/姿勢補償の過渡として効いている可能性が高く、低いCOG目標・関節過渡・fc低下と同時に発生している。

## 改善案

1. link4 anchorのCOG目標に最低高度制約を入れる。少なくとも `target_pos_z` が安全高度を下回る場合は形状変化をholdまたはscaleする。
2. `link4_anchor_max_body_pos_rate` をz方向にも明示的に厳しくし、`nav_z_rate` の下向き最大値を制限する。
3. 関節角速度、特に `joint2_pitch/joint2_yaw` の指令速度をshape gateの入力に加える。現在のfc gateは静的形状中心で、link4 anchorが生成するCOG目標過渡を見ていない。
4. 実測fc warning状態、特に torque safety scaleが低い間は、link4 anchorの低高度方向の補償を抑える。
5. gimbal速度またはgimbal追従誤差が大きい場合は、次の形状更新をholdする。静的fcが十分でもallocation過渡中は実効的な余裕が小さい。
