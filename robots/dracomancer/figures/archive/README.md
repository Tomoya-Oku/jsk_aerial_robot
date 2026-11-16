# 過去の実験結果

保存対象とした図・CSV・集計を元のフォルダ単位で管理する。
現行Mk-IIの検証完了を示す資料とは扱わない。

| フォルダ | 内容 |
|---|---|
| [fc_thresholds_4096grid](fc_thresholds_4096grid/) | 旧force-margin計算による4096形状のCSV・図・集計 |
| [fc_thresholds_4096grid_signed](fc_thresholds_4096grid_signed/) | 符号付きforce-margin計算のCSV・図・集計・再生成手順 |
| [joint_angles_mapping](joint_angles_mapping/) | 過去の関節写像・位置の図と集計 |
| [rsj2026_test1](rsj2026_test1/) | RSJ 2026向けに作成された図 |
| [shape_test_20260706_175417](shape_test_20260706_175417/) | 過去の形状操作試験の図・PDF・集計 |

集計内の出力パスは生成当時の場所を示し、現在の保存先とは異なる場合がある。
作図コードは[figures/scripts/](../scripts/)に置く。再生成結果は通常の`figures/`配下や
外部の結果保存先へ出力し、保存対象と決めたものだけを本フォルダへ移す。
アーカイブはCatkinのインストール対象に含めない。
