# Dracomancer Mk-II / 形態協調型テレオペレーション
## 最終仕様書・設計書

> **主題**
> 人間の上肢形態を用いて、細長い多関節飛行ロボットの内部形状を直感的に操作し、
> 飛行安定性・狭隘空間との干渉をロボット側で補正する **Human-in-the-loop morphology teleoperation framework** を構築する。

---

> **正本の構成**
> 本ファイルは正本の入口です。仕様本文は以下の章ファイルで管理し、変更時は該当章だけを更新します。各章末には、その章から直接参照する参考文献を収録しています。

## 章一覧

1. [1. 研究背景](design-spec/01-background.md)
2. [2. 研究目的](design-spec/02-objectives.md)
3. [3. 概要（コンセプト）・システム全体像](design-spec/03-concept.md)
4. [4. デバイス設計（ハードウェア）](design-spec/04-hardware.md)
5. [5. システム設計（ソフトウェア）](design-spec/05-software.md)
6. [6. 検証（実験）環境・実施タスク・評価項目](design-spec/06-evaluation.md)
7. [7. 先行研究の採用箇所と本研究との差分](design-spec/07-related-work.md)
8. [8. 実装優先順位](design-spec/08-priorities.md)
9. [9. 現時点で要確定の仕様](design-spec/09-open-issues.md)
10. [10. 最終的な研究の主張](design-spec/10-claims.md)

## 編集規則

- 仕様・設計・研究計画の正本は、本ファイルと `design-spec/` 配下の章ファイルです。
- 同じ内容をREADMEや別の文書へ複製しません。必要な場合は該当章へのリンクだけを記載します。
- 章内で文献を追加・削除した場合は、その章末の「参考文献」も同時に更新します。
- 複数章に関係する文献は、各章から単独で参照できるよう、それぞれの章末に再掲します。
