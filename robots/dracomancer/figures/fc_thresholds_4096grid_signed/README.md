# Signed fc_min 4096-grid results

This directory contains plots generated from the DRAGON Gazebo 4096-grid
rerun using the signed force-margin calculation.

- Source commit: `44fe7a58`
- Samples: 4096 (`grid_0000` through `grid_4095`)
- Missing / duplicate / non-finite rows: 0 / 0 / 0
- CSV SHA-256:
  `17dda4605bf0b100e3f40fbdff34113b937fedc9618778ceb740d4788e9116e8`

## Main plots

- `fc_4096grid_signed_force_distribution.{png,svg}`: recommended force-margin
  view. Negative and non-negative margins are color-coded, the central
  `[-1.5, 2.5] N` range contains 4044 of 4096 samples, and an inset separates
  the two closely spaced force thresholds.
- `fc_4096grid_signed_distribution.{png,svg}`: complete signed range,
  including the minimum force margin of `-33.957 N`.
- `fc_4096grid_signed_zoom_distribution.{png,svg}`: central force range
  `[-2.5, 2.5] N`; the 40 samples outside that range are annotated.
- `fc_4096grid_signed_by_sample.{png,svg}`: values in grid traversal order.
- `fc_4096grid_signed_force_vs_torque.{png,svg}`: force/torque relationship.
- `fc_4096grid_signed_by_joint.{png,svg}`: relationship with commanded joints.
- `fc_4096grid_signed_altitude_correlation.{png,svg}`: relationship with altitude.

The previous unsigned results remain in the sibling
`fc_thresholds_4096grid` directory.

## Threshold summary

Using the existing rule `hard_min = max(0, p5)` and
`min = median * 0.8`:

| Quantity | p5 | median | hard_min | min threshold |
| --- | ---: | ---: | ---: | ---: |
| Force [N] | -0.667459 | 0.034458 | 0.000000 | 0.027566 |
| Torque [N m] | 0.017600 | 0.358444 | 0.017600 | 0.286756 |

The force margin was negative for 1910 shapes. These proposed force values
have not been applied to runtime configuration because the old percentile
heuristic needs to be reconsidered for a signed distribution.

## Regeneration

```bash
MPLCONFIGDIR=/tmp/mpl_fc_signed \
python3 ../../scripts/experiments/plot_fc_data.py \
  fc_4096grid_signed_merged.csv \
  --out-dir . \
  --name-prefix fc_4096grid_signed \
  --formats png svg \
  --bins 60 \
  --plots distribution scatter sequence joint altitude \
  --label-threshold-values
```

```bash
MPLCONFIGDIR=/tmp/mpl_fc_signed_zoom \
python3 ../../scripts/experiments/plot_fc_data.py \
  fc_4096grid_signed_merged.csv \
  --out-dir . \
  --name-prefix fc_4096grid_signed_zoom \
  --formats png svg \
  --bins 60 \
  --plots distribution \
  --force-range -2.5 2.5 \
  --torque-range 0 2.5 \
  --label-threshold-values
```

```bash
MPLCONFIGDIR=/tmp/mpl_fc_signed_force \
python3 ../../scripts/experiments/plot_fc_data.py \
  fc_4096grid_signed_merged.csv \
  --out-dir . \
  --name-prefix fc_4096grid_signed \
  --formats png svg \
  --bins 64 \
  --plots force \
  --force-range -1.5 2.5
```
