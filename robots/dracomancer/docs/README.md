# Dracomancer Mk-II ドキュメント

現行実装は`dracomancer`パッケージのMk-II専用構成。標準名前空間は`/dracomancer`。
旧Mk-I、CAN経由の旧力覚出力、独立パッケージ`dracomancer_mkii_ros`は使用しない。

## 操作・検証

| 文書 | 内容 |
|---|---|
| [usage.md](usage.md) | ビルド、viewer・Gazebo・実機の起動、検証コマンド |
| [spinal.md](spinal.md) | H7 V2の2バス実装と事前調査 |
| [hardware.md](hardware.md) | 実機の校正、トルク操作、停止・復旧 |
| [shape.md](shape.md) | DRAGON形状操作の起動、設定、診断・ログ |

## 理論と実装

制御理論は修士論文を正本とし、以下は実装・操作の資料とする。
[修論との対応](thesis.md)で原稿の参照時点と実装範囲を確認する。

| 文書 | 内容 |
|---|---|
| [research.md](research.md) | 研究背景・目的・仮説、操作の分担、先行研究との差分 |
| [device.md](device.md) | 機械設計、サーボ・ID・通信仕様、設計上の制約 |
| [software.md](software.md) | 現行の制御経路・数理仕様と、未実装機能の設計 |
| [evaluation.md](evaluation.md) | 比較実験・統合タスクと評価項目 |
| [roadmap.md](roadmap.md) | 残る開発・校正・検証と未確定仕様 |

実装済みの処理と研究上の到達目標を区別する。運用手順からは設計の該当箇所を参照し、
仕様・設定値の説明を重複させない。完了済みの実装指示書、ブランチ調査・移行履歴、
統合前の性能測定値は削除した。過去の状態はGit履歴を参照する。
