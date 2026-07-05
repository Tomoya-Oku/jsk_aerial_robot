# commit eeb9efe test analysis

解析対象:

- `/home/oku/rosbag/commit_eeb9efe08666091af6b26470bbb533ee9e7b22bd_test_20260705_235013.bag`
- bag duration: 136 s
- 対応コード: `eeb9efe0` 時点の fc 予測改善後、joint3_pitch 90deg オフセット削除前

生成物:

- `robots/dracomancer/figures/eeb9_test/safety_overview.{png,svg}`
- `robots/dracomancer/figures/eeb9_test/fc_prediction_error.{png,svg}`
- `robots/dracomancer/figures/eeb9_test/joint_mapping.pdf`
- `robots/dracomancer/figures/eeb9_test/joint*.{png,svg}`

## 結論

- このbagでは `/dragon/flight_state` は全区間 `5` のままで、明示的な failsafe / force landing ログは記録されていない。
- ただし実測高度は bag 開始後 `96.992 s` に `z < 0.3 m` まで落ちており、操縦中に一度大きく沈下している。
- 予測 fc は候補・最終targetとも hard threshold を下回らず、feasibility gate はほぼ常時 `deform=True` と判断している。
- 一方、実測 torque fc は hard threshold 近傍または未満まで頻繁に落ちる。特に沈下直前は実測 torque fc が `0.00255` まで落ちているのに、予測 target torque fc は `0.197` と見積もっている。
- force fc は予測が実測より大きく、同時刻補間では target 予測の force MAE が `1.160`、RMSE が `1.306`。torque は target 予測の MAE が `0.292`、RMSE が `0.369`。

## fc比較

hard threshold:

| channel | hard threshold |
| --- | ---: |
| force | 0.10899 |
| torque | 0.01540 |

全区間の最小値:

| signal | min | max | mean |
| --- | ---: | ---: | ---: |
| candidate force fc | 0.737 | 2.405 | 1.783 |
| target force fc | 0.824 | 2.405 | 1.772 |
| measured force fc filtered | 0.034 | 6.038 | 1.034 |
| candidate torque fc | 0.042 | 0.783 | 0.415 |
| target torque fc | 0.018 | 0.782 | 0.416 |
| measured torque fc filtered | 0.000036 | 1.792 | 0.382 |

同時刻補間での誤差:

| prediction | measured | bias | MAE | RMSE |
| --- | --- | ---: | ---: | ---: |
| candidate force | measured force | 0.741 | 1.170 | 1.313 |
| target force | measured force | 0.730 | 1.160 | 1.306 |
| candidate torque | measured torque | 0.029 | 0.292 | 0.368 |
| target torque | measured torque | 0.029 | 0.292 | 0.369 |

bag開始後 `96.992 s` の `z < 0.3 m` 直前:

| signal | last value before event |
| --- | ---: |
| target force fc | 1.093 |
| measured force fc filtered | 0.295 |
| target torque fc | 0.197 |
| measured torque fc filtered | 0.00255 |

## joint3_pitch

このbagは joint3_pitch の 90deg オフセット削除前なので、記録内では概ね
`joint3_pitch = clamp(shoulder_flexion_extension_joint + pi/2)` になっている。

| signal | min | max | mean |
| --- | ---: | ---: | ---: |
| Dracomancer shoulder flexion | -0.896 | 0.172 | -0.170 |
| candidate joint3_pitch | 0.675 | 1.571 | 1.271 |
| commanded joint3_pitch | 0.019 | 1.571 | 1.252 |

bag開始後 `43 s` 付近で `joint3_pitch` が上限 `1.57 rad` へ急に入り、その後も肩屈曲が負側に大きくなるほど `joint3_pitch` は正側へ残る。今回の実装変更では `distal_offsets[3] = 0` としたため、同じDracomancer入力なら joint3_pitch は肩屈曲値に直接近づき、この正側飽和は避けられる見込み。

## 改善メモ

- 予測fcを実測fcへ近づけるには、shape gate の単純なモデル出力だけでなく、DRAGON controller の allocation 結果または実測 `/dragon/debug/fc_*` の低下をgateに反映する必要がある。
- 今回のbagでは target 予測と candidate 予測の差は小さく、候補姿勢と最終送信姿勢の差より、controller内部状態・gimbal実応答・姿勢/高度過渡・低通過後の実測fcとの差が支配的。
- gateを hard threshold だけで判定すると、実測 torque fc が短時間で0近傍へ落ちる挙動を止められない。recover threshold または実測fc低下時の一時holdを併用する方が実機保護としては有効。
