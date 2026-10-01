# 7. 先行研究の採用箇所と本研究との差分

| 先行研究 | 採用する成果 | 本研究で新たに行う部分 |
|---|---|---|
| Zhao et al. [1] | DRAGON機構、joint range、flight controller、variable rotor configuration | Human morphologyから形態commandを生成 |
| Anzai et al. [2] | \(f_{min},\tau_{min}\)、thrust / joint constraints、形態最適化枠組み | Human shapeを最優先にしたfeasibility filterとして再構成 |
| Kaneko et al. [3] | articulated aerial robot teleoperationの課題、joint command、hybrid joystickの示唆 | floating-baseをやめ、装着型arm morphology + joystickに分離 |
| Chen et al. [4] | point-cloud collision checking / multi-link geometry処理 | global planningを使わずLiDAR local safety constraintとして利用 |
| Aerial haptics [5][6] | contact / force feedbackの基本構造 | articulated morphology mappingを介した上肢関節へのfeedback |
| Exoskeleton design review [8][9] | self-alignment、attachment、adjustability、低負荷設計 | teleoperation入力・力覚用途へ具体化 |

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
