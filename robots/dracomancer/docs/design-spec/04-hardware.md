# 4. デバイス設計（ハードウェア）

## 4.1 基本構成

主デバイスは **右腕7 DoFの上肢外骨格型 Dracomancer Mk-II** とする。

主用途は、

- 人体関節状態の計測
- 腕リンク方向・相対姿勢の取得
- 将来的な関節トルク提示

である。

位置・姿勢入力装置として腕全体を空間中で大きく移動させる必要はない。

---

## 4.2 従来デバイスからの変更点

### Kanekoらのfloating-base deviceとの差分

Kanekoらの装置では、両手でデバイス全体を保持し、

- device position → robot position
- device attitude → robot attitude
- device joint angles → robot joint angles

として全自由度を入力した [3]。

本研究ではこの設計をそのまま継承せず、

| 項目 | Kaneko et al. | Dracomancer Mk-II |
|---|---|---|
| 装着方式 | 両手保持 floating-base | 身体装着型 |
| 基本姿勢 | 両手でデバイスを保持 | 座位を基本 |
| Base position | デバイス移動 | Joystick |
| Base attitude | デバイス回転 | Joystick等 |
| Internal shape | device joints | 上肢形状 |
| 長時間疲労対策 | floating-baseのため課題 | 身体支持・省動作 |
| 入力間干渉 | position / attitude / joints間で発生 | 物理的に入力チャネル分離 |
| Haptic | 未実装 | Mk-IIで拡張 |

Kanekoら自身も、長時間作業では floating-base device の保持負荷が問題となり、Joystickとのhybrid利用が有効な可能性を指摘している [3]。
したがって、**Joystickとの分離は先行研究の課題を踏まえた設計変更**である。

---

## 4.3 Mk-Iからの機械設計変更

Mk-IIでは、肩固定＋手先把持のみの構成から、人体と外骨格のずれを抑えるために以下を導入する。

### 必須

- 上腕カフ
- 前腕カフ
- 上腕・前腕リンク長調整機構
- 肩基準位置調整機構
- 軽量リンク
- ケーブル取り回しの改善

### 上腕・前腕ロール対策

単純な固定カフでは人体の軸方向回転を拘束するため、

> **カフと外骨格リンクの間に受動回転自由度または回転スライド機構を設ける**

ことを基本方針とする。

これにより、

- 腕と外骨格の密着性
- 上腕ロール計測
- 装着時の関節軸ずれ低減

を両立させる。

Rigid exoskeletonでは人体関節軸とのmisalignment、attachment interface、身体寸法への適応が装着性に大きく影響することが報告されている [8]。本研究では新しい人間工学理論を作らず、これらの既知の設計原則を採用する。

---

## 4.4 軽量・低慣性設計

設計優先順位を

1. 末端慣性
2. 関節摩擦
3. 総質量

とする。

方針：

- 重量物は肩・体幹側へ寄せる
- 前腕・手先側は最小限の構造とする
- 中空アルミ / CFRP等を使用
- 不要な外装を設けない
- 配線重量も設計対象とする

評価では総質量だけでなく、

> **肩関節回りの等価慣性**

を用いる。

---

## 4.5 ユニバーサルデザイン

調整対象：

- 上腕リンク長
- 前腕リンク長
- 上腕カフ位置
- 前腕カフ位置
- 肩基準位置
- カフ径

候補機構：

- Telescopic link
- 長穴＋クランプ
- Index plunger
- Quick release

最終調整範囲は人体寸法データとMk-I実測値から決定する。

### 確認が必要な数値仕様

- 総質量目標
- 前腕部最大質量
- 上腕リンク調整範囲
- 前腕リンク調整範囲
- カフ径
- 装着時間

現時点では根拠のない数値を設定せず、Mk-I計測と人体寸法データから決定する。

---

## 4.6 Actuator / Sensor

### センサ

各関節について最低限、

- joint angle
- joint velocity

を取得する。

### Actuator

力覚提示を行う関節には、バックドライブ性・通信周期・トルク余裕を確認した smart servo を使用する。
既存部品・既存ドライバを優先し、新規モータ制御基板は原則開発しない。

---

## 4.7 Dual-armについて

Dual-arm 14 DoF構成は理論上維持するが、**修士研究の必須実装から外す。**

Single-armで

- 6D DRAGON morphology coverage
- 狭隘通過
- 接触タスク

が成立しない場合のみ Phase 2 として導入する。

理由は、主要な研究課題が mapping representation と feasibility-aware retargeting であり、Dual-armまで同時に実装すると評価条件が過剰に増えるためである。

---

## 参考文献

[3] K. Kaneko, J. Sugihara, K. Sugihara, M. Kitagawa, K. Nagato, and M. Zhao,
“A Teleoperation Framework for an Articulated Aerial Robot with Full DoF Mapping of Base Pose and Joint Angles,”
*2026 IEEE/SICE International Symposium on System Integration (SII)*, pp. 245–250, 2026.
DOI: 10.1109/SII64115.2026.11404691.

[8] L. Chen, D. Zhou, and Y. Leng,
“A Systematic Review on Rigid Exoskeleton Robot Design for Wearing Comfort: Joint Self-Alignment, Attachment Interface, and Structure Customization,”
*IEEE Transactions on Neural Systems and Rehabilitation Engineering*, vol. 32, pp. 3815–3827, 2024.
DOI: 10.1109/TNSRE.2024.3479283.
