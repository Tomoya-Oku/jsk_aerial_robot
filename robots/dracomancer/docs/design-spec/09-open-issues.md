# 9. 現時点で要確定の仕様

以下は実測・予備実験後に決定する。

1. Mk-II総質量目標
2. 上腕・前腕リンク調整範囲
3. カフ径・カフ構造
4. Link 4用人体仮想基準点
5. Human–Robot global shape errorの最終定義
6. \(f_{min},\tau_{min}\) の安全threshold
7. LiDAR minimum clearance \(d_{safe}\)
8. DRAGON安全joint velocity上限
9. Position / attitude joystick allocation
10. XM430-W350-R各関節の最大提示トルク、電流上限、符号、実トルク換算値と閉ループ安定性評価（直結経路・校正ゲートは[4.6](04-hardware.md#46-actuator--sensor)を参照。shapeの接触Jacobianと力覚出力への接続は未実装）
11. External wrench estimatorを使用可能か
12. Reference linkの標準設定

2026-11-16の形状マッピングデモでは、Link4用人体仮想基準点をbase_link原点とする
シミュレーション専用仮設定を使用する。人体代表点はCADフレーム/手先重心の代理点であり、
identityの固定軸合わせと共に実機用の確定仕様ではない。
DRAGON発行指令の速度上限は現行URDFの0.1rad/sを採用するが、項目8の実機安全速度は未確定のままとする。

---

## 参考文献

この章で直接参照する文献はありません。
