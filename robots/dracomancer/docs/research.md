# 研究計画

## 背景と目的

DRAGONは4リンク・3関節×Pitch/Yawの内部6自由度と、推力方向を変えるロータ構成を持つ [1]。
形態変更を利用した狭隘空間通過や接触作業を目指せる一方、内部関節と機体全体を同時に操作する負担がある。
floating-base型デバイスによる全自由度操作では、入力間の干渉、保持疲労、接触状態の把握が課題となる [3]。

本研究は、**人間の上肢形態をロボット内部形状へ写像し、飛行・環境制約の範囲で操作者の形状意図を維持する**
形態協調型テレオペレーションを目指す。既存の飛行可能性評価 [2] とpoint-cloud幾何処理 [4] を再利用する。
人間が進行方向・通過する隙間・作業に適した基本形状を決め、ロボットが安定化・制約評価を担当する。

## 入力と役割の分担

| 操作・情報 | 担当 |
|---|---|
| 内部形状と変形のタイミング | 右腕Dracomancer Mk-II |
| 機体の位置・必要な姿勢自由度 | Joystick等の独立入力 |
| 形状指令の保持 | Clutch／操作モード |
| 局所障害物 | DRAGONのLiDARを利用する計画 |
| 接触状態 | wrench estimatorを評価し、不十分ならF/Tセンサを追加する計画 |

座位での操作を基本とし、腕全体の世界座標における並進・回転を機体の移動指令にしない。
DRAGONの内部座標は`[joint1_pitch, joint1_yaw, joint2_pitch, joint2_yaw, joint3_pitch, joint3_yaw]`。
実際の関節境界は読込URDFを使う。

目標とする経路は「上肢形状→写像→人間の希望形態→飛行可能性・局所衝突制約→採用形態」。
現在実装されている範囲と後続の制約付き補正は[ソフトウェア設計](software.md)で区別する。

## 検証仮説

以下は検証する仮説であり、実装完了や実験による証明を意味しない。

| 仮説 | 検証内容 |
|---|---|
| H1：形態入力 | 腕の相対形状が関節個別操作より連続的・直感的な内部形態操作を可能にするか |
| H2：写像表現 | 修論の片腕・両腕それぞれの関節角方式と中心線方式で、形状再現性・操作特性がどう異なるか |
| H3：飛行可能性 | 人間の形状意図を維持しながら、推力飽和や危険形態を抑えられるか |
| H4：環境制約 | 人間の経路判断を維持した局所衝突制約が、狭隘部での衝突リスクを減らせるか |
| H5：力覚（拡張） | 接触・制約情報を上肢へ返すことで、視覚だけの場合より状態を把握しやすくなるか |

比較条件・評価指標は[実験計画](evaluation.md)にまとめる。

## 研究範囲

理論の範囲は[修論](thesis.md)に従い、装着型ハードウェア、片腕・両腕の4写像、
ヒステリシス付きhold、接触・制約力覚、Joystickと尾側／中央基準点の補償を扱う。
LiDAR局所制約は追加研究の案であり、現在の修論第4章の制約判定に実装済みとは扱わない。

Web操作、ROS bagの再生と動画・静止画出力は運用・成果説明の補助機能で、主要な学術的寄与に含めない。
新しい飛行ロボット機構・飛行制御器・SLAM・意味地図・自律経路計画は開発対象外。
DRAGON全体のROS 2移植も研究の完了条件とせず、既存DRAGONが安定する環境を優先する。

一般化の対象は、多リンク構造、内部一般化座標、順運動学、制約評価を定義できるロボットとする。
現在の4リンクDRAGON実装だけで、他機体への一般化や全自由度の可逆な写像を実証したとは扱わない。

## 先行研究の採用箇所と差分

| 先行研究 | 採用する成果 | 本研究で検証する部分 |
|---|---|---|
| Zhaoら [1] | DRAGON機構、関節範囲、飛行制御器、可変ロータ配置 | 上肢形状からの内部形態指令 |
| Anzaiら [2] | 飛行可能性指標、推力・関節制約、形態最適化 | 人間形状を主目標とする補正 |
| Kanekoら [3] | 多関節飛行ロボット遠隔操作の課題、Joystick併用の示唆 | 装着型形態入力と機体移動の分離 |
| Chenら [4] | point-cloud衝突判定・多リンク幾何処理 | 局所安全制約への利用。大域経路は人間が選ぶ |
| Aerial haptics [5][6] | 接触・力覚フィードバックの基本構造 | 形態写像を介する上肢関節への提示 |
| Exoskeleton review [8][9] | 軸合わせ、装着面、調整機構、低負荷設計 | 遠隔操作・力覚デバイスとしての具体化 |

最終的な主張は、入力分離、A/B写像比較、人間の意図を優先する実現可能性補正の3点とする。
単体ソフトウェアの動作確認を、操作性・安全性・飛行安定性の実証に置き換えない。

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

[5] M. Allenspach, N. Lawrance, M. Tognon, and R. Siegwart,
“Towards 6DoF Bilateral Teleoperation of an Omnidirectional Aerial Vehicle for Aerial Physical Interaction,”
*IEEE International Conference on Robotics and Automation (ICRA)*, pp. 9302–9308, 2022.

[6] M. Macchini, T. Havy, A. Weber, F. Schiano, and D. Floreano,
“Hand-Worn Haptic Interface for Drone Teleoperation,”
*IEEE International Conference on Robotics and Automation (ICRA)*, pp. 10212–10218, 2020.

[8] L. Chen, D. Zhou, and Y. Leng,
“A Systematic Review on Rigid Exoskeleton Robot Design for Wearing Comfort: Joint Self-Alignment, Attachment Interface, and Structure Customization,”
*IEEE Transactions on Neural Systems and Rehabilitation Engineering*, vol. 32, pp. 3815–3827, 2024.
DOI: 10.1109/TNSRE.2024.3479283.

[9] F. Nazari, N. Mohajer, D. Nahavandi, A. Khosravi, and S. Nahavandi,
“Applied Exoskeleton Technology: A Comprehensive Review of Physical and Cognitive Human–Robot Interaction,”
*IEEE Transactions on Cognitive and Developmental Systems*, vol. 15, no. 3, pp. 1102–1122, 2023.
DOI: 10.1109/TCDS.2023.3241632.
