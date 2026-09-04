# 1. 研究背景

多関節飛行ロボットは、複数リンクと内部関節を持つことで、一般的な剛体型マルチロータでは困難な

- 細長い形態への変形による狭隘空間通過
- ロータ配置・リンク配置の変更
- End Effector（EE）を用いた接触作業
- リンク長を利用した大きなモーメント生成

などを実現できる可能性がある。

DRAGON は4リンク・3内部関節から構成され、各関節が Pitch / Yaw の2自由度を持つことで、6 DoF の内部形状を空中で変更できる。また各リンクの dual-rotor gimbal module により推力方向を変更し、形態変化に伴う可制御性低下を緩和する設計が採用されている [1]。

一方、多関節飛行ロボットの自由度増加は、そのまま操作自由度の増加につながる。Kanekoらは、両手で保持する floating-base 型デバイスを用い、デバイス全体の位置・姿勢と内部関節角をロボットの base pose・joint angles に対応させることで、全自由度を同時操作する枠組みを示した [3]。しかし同研究では、

- 位置だけを操作したい場合でも姿勢・関節角指令が意図せず発生する
- floating-base デバイスを長時間保持することによる疲労
- 接触力を操作者が把握できない

といった課題が報告されている。

本研究では、これらを踏まえ、**「全自由度を一つの身体動作へ割り当てる」ことを目的としない。**
代わりに、

- **腕の相対形状 → ロボット内部形状**
- **Joystick → ロボット全体位置・姿勢**

と入力チャネルを分離する。

これにより、操作者は座位で大きく姿勢を変えることなく、上肢を「ロボットの可変形状入力器」として使用できる。

さらに、人間が入力した形状をそのまま実行するだけでは、形態によって

- 推力飽和
- 制御特異点
- 関節トルク制約
- 障害物との干渉

が生じる。

Anzaiらは、多関節飛行ロボットに対して、目標関節角に近い形を維持しながら guaranteed minimum force / torque、推力範囲、関節トルクなどを考慮した形態最適化を示している [2]。また Chenらは、raw point cloud を直接利用し、衝突回避と動的実現可能性を考慮した多関節 floating-base robot の軌道計画を示している [4]。

本研究ではこれらの先行研究を新たに作り直さず、**既存の飛行可能性評価・point-cloud collision checking を最大限再利用する。**
研究の新規部分は、これらを **人間形状入力のための安全・実現可能性フィルタとして統合すること** に置く。

---

## 参考文献

[1] M. Zhao, T. Anzai, F. Shi, X. Chen, K. Okada, and M. Inaba,
“Design, Modeling, and Control of an Aerial Robot DRAGON: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation,”
*IEEE Robotics and Automation Letters*, vol. 3, no. 2, pp. 1176–1183, 2018.
DOI: 10.1109/LRA.2018.2793344.

[2] T. Anzai, M. Zhao, M. Murooka, F. Shi, K. Okada, and M. Inaba,
“Design, Modeling and Control of Fully Actuated 2D Transformable Aerial Robot with 1 DoF Thrust Vectorable Link Module,”
*2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2820–2826, 2019.

[3] K. Kaneko, J. Sugihara, K. Sugihara, M. Kitagawa, K. Nagato, and M. Zhao,
“A Teleoperation Framework for an Articulated Aerial Robot with Full DoF Mapping of Base Pose and Joint Angles,”
*2026 IEEE/SICE International Symposium on System Integration (SII)*, pp. 245–250, 2026.
DOI: 10.1109/SII64115.2026.11404691.

[4] Y. Chen, J. Li, H. Liu, Z. Luo, K. Kaneko, and M. Zhao,
“Hierarchical Trajectory Planning of Floating-Base Multi-Link Robot for Maneuvering in Confined Environments,”
*IEEE Transactions on Automation Science and Engineering*, 2026.
DOI: 10.1109/TASE.2026.3669051.
