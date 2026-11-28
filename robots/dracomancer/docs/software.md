# ソフトウェア設計

## 現行実装

ROS 1の`dracomancer`パッケージにモデル・GUI・直結制御・形状写像を集約する。
起動・設定は[操作手順](usage.md)、物理構成は[デバイス設計](device.md)を参照する。

| 機能 | 実装と対応範囲 |
|---|---|
| 表示・角度／トルク操作 | `scripts/mkii/control.py`・`panel.py`。viewerとGazebo |
| 実機入出力 | `scripts/mkii/direct.py`、校正済み換算`src/dracomancer/xm430.py` |
| 未校正の実機仮表示 | `scripts/mkii/observe.py`。起動姿勢基準の読み取り専用表示。形態制御には使用しない |
| 形状写像B | `scripts/control/shape_mapping.py`・`shape_runtime.py` |
| 関節角写像 | `scripts/control/joint_mapping.py`。修論の片腕・両腕方式 |
| 形態指令の所有者 | `scripts/control/control_joint_angle.py`。`shape`・`joint`・`dual_shape`・`dual_joint` |
| 飛行可能性評価 | `src/shape_feasibility_node.cpp`と`ShapeFeasibility.srv`。候補・発行形態の判定とhold |
| 機体移動・姿勢 | `control_position.py`と`navigation.py`。視点・体幹傾きによる2軸入力面。Joystick操作は既定無効 |
| 壁接触の観測 | `wall.py`・`src/wall_contact.cpp`。シミュレーション用 |
| 力覚 | `feedback.py`からNmを送信。既定無効、自動ARMなし。4方式の局所Jacobianに対応 |
| 基準点補償 | `anchor_runtime.py`。尾側点または中央関節を基準とし、`control_position.py`だけが移動指令を発行 |

理論は[修論](thesis.md)を優先する。可行集合への射影やLiDAR制約を追加した最適化ではなく、
修論のヒステリシス付きholdと速度制限・再判定を実装する。

## 上肢形状の取得

入力はCAD関節名を持つ`measured_joint_states`。名前・有限値・時刻・必要関節の充足を確認してFKへ渡す。
実機では校正済み・新鮮な計測関節だけを元の計測時刻で配信する。表示用`joint_states`の未計測q=0を入力にしない。
viewerは仮想関節状態、Gazeboはシミュレータの元計測時刻を同じトピックへ配信する。

上腕・前腕・手先方向、隣接リンク姿勢、曲げ方向、腕全体の正規化形状を表現対象とする。
世界座標における腕全体の並進・回転は機体移動に使わない。

## 全体形状写像B

Human chainは固定`base_link`基準で計算し、肩の相対回転を保持する。
DRAGON chainは実URDFの関節列を計算してLink4尾端基準へ変換し、Link1に向かう順序へ反転する。
4本のリンクと3本のPitch/Yawスペーサを含む物理端点を使う。

両中心線を開始点で並進除去し、総長で正規化して等弧長サンプルを取る。
これは片腕の場合である。`dual_shape`では左手→左肩→右肩→右手をつなぎ、
人体は左右肩の中点、機体は第2関節pitch軸の位置を引いてから全長で正規化する。
固定装着姿勢の軸合わせには固定回転だけを使い、動く肩関節の回転を相殺しない。
中心線サンプルを\(\hat p_H(s_k;q_H)\)、\(\hat p_D(s_k;q)\)とし、次を最小化する。

\[
E_{shape}(q)=\frac{\sum_k w_k\|\hat p_D(s_k;q)-\hat p_H(s_k;q_H)\|^2}{\sum_k w_k},
\qquad
q_{ref}=\arg\min_{q\in\mathcal Q_{joint}}
\left(E_{shape}(q)+\lambda\|q-q_{prev}\|^2+\rho\|q-q_{rest}\|^2\right).
\]

`q_prev`は最後に発行した指令、`q_rest`は明示した参照形態（未指定ならstartup pose）。
\(\mathcal Q_{joint}\)は読込DRAGON URDFの関節境界。現在は位置形状誤差を使い、接線・曲率・手先姿勢は目的関数に含めない。
有限差分ヤコビアン、減衰付き最小二乗、境界への投影、バックトラッキング、前回解からのwarm startを用いる。

設定の正本は`config/control/shape.yaml`。既定は20点、最適化10 Hz、最大1反復、時間予算20 ms、指令ループ40 Hz。
最適化結果を周期内・更新周期間で共有し、診断のために解を再度進めない。
体幹点・人体代表点・固定軸合わせの設定方法は[形状操作](shape.md)にまとめる。

中心線は形状の表現次元を増やすが、人体の独立自由度を増やさない。
純粋な軸回旋や手先姿勢が中心線に現れない場合があり、人体7軸をDRAGON6軸へ可逆に写せるとは扱わない。

## 関節角方式

`joint`は修論の手首回転行列・肘のpitch/yaw配分・肩角の直接対応を用いる。
`dual_joint`は右肘→第1関節、校正済み肩角の右−左→第2関節、左肘→第3関節。
基準姿勢では`rest`と一致し、手首と前腕回内外を両腕の直接対応に使わない。
`shape/joint_mapping`でCAD名、解剖学的角度への零点・符号尺度、両腕の基準角と対角係数を設定する。
既定値はシミュレーション用の代理値で、実時間での関節角方式は`calibrated: true`を要求する。

## 飛行可能性と指令保持

現在の経路は「形状候補→予測サービス→holdゲート→速度制限→発行形態の再検査→`/dragon/joints_ctrl`」。
予測は`valid`・`stability_ok`とforce/torque余裕を確認し、拒否後の復帰にはヒステリシスを使う。
最適化失敗、入力異常、予測サービス障害、余裕不足、時間超過では最後の発行指令を保持する。
初回は、新鮮な実関節姿勢と設定・可行性の確認がそろうまで発行しない。

発行指令は`max_step`と経過ROS時間に基づく軸別速度上限で制限し、URDF上限とも交差させる。
既定0.1 rad/sは現行URDF由来で、実機の安全値ではない。
時計停止・巻戻り・再開後に遅延分を加算しない。実関節の追従・外力下の速度は別に評価する。

`force_dimensions:=2`が標準のXY評価で、`3`はXYZ評価。推力偏向角とモデル姿勢を固定した同一Qで、
射影領域の面までの符号付き距離と、上下限内の共通推力配分を評価する。
希望形態の評価コンテキストをサービスの応答から引き継ぎ、速度制限後の再判定でも用いる。
再判定で拒否された場合も保持状態へ入り、次は復帰しきい値を要求する。
配分計算は有界最小二乗の残差を検証し、反復上限時は保守的に拒否する。
`use_contact_wrench:=true`では新鮮なlink1原点まわりの接触レンチを用い、重心へのモーメント移動を含めて静的釣合いを判定する。
力・トルクの評価空間に応じた閾値は実測校正が必要で、XY用の値をXYZの検証済み値とは扱わない。

現在の離散的な候補判定は、経路全体の安全・動的安定性・接触力学や、可行集合への最近点射影を保証しない。
速度制限後の形態を再検査しても、全中間点の検査にはならない。
初期DRAGON研究の速度条件 [1] も、現在の実機へ無検証で一般化しない。

## Joystick・モード

主な複合操作は「右腕→内部形状、Joystick→機体移動」。通常の形態指令はHover条件を使う。
修論の改稿に従い、視点座標とIMUの相対傾きから入力面を作り、2軸のスティックを3次元の速度候補へ写す。
両腕では右スティックが並進、左が回転。片腕では押下後の中立確認で切り替える。
姿勢は回転行列で積分し、位置・姿勢の目標を切替や体幹基準の再設定でリセットしない。
操作者位置・左右トピック・旧入力経路は[修論対応](thesis.md#視点基準の並進回転)を参照する。
`teleop_mode=false`は最後の発行形態を保持し、現在の実装はclutch解除時の人体基準の再設定を行わない。
Joystickだけの長距離移動、機体位置を保つ形態操作、基準リンク固定を研究上の操作形態とし、
現在の対応と未実装のモード遷移を[残作業](roadmap.md)で区別する。
地上デモでHover条件を外す構成は[形状操作](shape.md)に限定して明示する。

## LiDAR局所衝突制約の設計

未実装。点群\(\mathcal P\)とロボット幾何からリンク距離、ロータ余裕、開口幅、接触・近接を評価し、
\(d(q_R,\mathcal P)\ge d_{safe}\)を局所制約に使う。
point-cloud処理と多リンク幾何処理 [4] を再利用し、大域経路・通過する隙間の選択は人間が担当する。

## 接触・安全力覚の設計

人体からロボットへの局所写像を\(M=\partial q_R/\partial q_H\)、
ロボット関節速度から接触点twistへの写像を\(J_R\)とし、
\(\tau_H^{contact}=M^TJ_R^Tw_R\)を基本とする。
接触wrenchと\(J_R\)の座標・モーメント基準点をそろえ、符号・遅延・ノイズを検証する。
既存wrench estimatorが不十分な場合にF/Tセンサを追加する。

安全制約通過前後の差\(e_R=q_R^{des}-q_R^{cmd}\)に対し、
\(\tau_H^{safe}=-M^TKe_R-B\dot q_H\)による抵抗・減衰を設計する。
各成分を合成し、トルク・変化率上限とdeadbandを適用する。基本構造は既存研究 [5][6][7] を参考にする。

`feedback.py`は同じ公称形態・入力・固定履歴でMとJを評価し、名前付きNm指令を実機の`haptic_torque`またはシミュレーションの`torque_cmd`へ送る。
複数接触では各点のJとレンチを個別に合成する。静的釣合いに加え、外部の摩擦・関節荷重検証が必要。
形状方式の数値差分は正負とも同一の公称解を初期値とし、保持・速度制限の微分は含めない。
基準点補償中は接触Jの並進行から基準点のJacobianを引き、合成Jacobianを使う。
`enable_haptics:=true`が必要。接触gainの既定値は0で、上限値も未校正のシミュレーション用設定。
計算遅延・入力断では実機への配信を停止し、電流セッションの期限でTorque OFFにする。
`haptics.launch simulation:=true`では接触wrenchを観測する。力覚の計算結果も配信する場合は`enable_haptics:=true`を指定する。
実機では`direct.py`が押下中の力覚指令を受け付け、校正済み`CurrentScale`、電流セッション、期限監視を使う。閉ループ安定性・受動性は未検証。
直結出力の準備・停止・復旧・排他条件は[実機手順](hardware.md)を参照する。

## Webとログ可視化の設計方針

研究を補助する設計であり、現在の標準bringupからWeb系を起動しない。
既存`aerial_robot_web`を再利用し、ブラウザUIとROS側gatewayを制御アルゴリズムから分離する。
状態、希望形態と採用形態、余裕・障害物距離、接続、モード・clutch、制約による保持を表示対象とする。
モード変更、低速な機体移動、実験開始・停止、ログ記録、緊急停止要求はROS側の共通入力経路を使う。

gatewayは許可commandを限定し、値検証、rate limit、timeout、明示enable、停止遷移を実装する設計とする。
ブラウザから制御topicへ直接publishせず、飛行・衝突制約を迂回しない。
ネットワーク公開時の認証・暗号化は利用環境を確定して決める。

ログの初期形式はROS 1 bag。入力形式の変換と再生データを分離し、
元ログhash・schema versionを保存する。機体姿勢・関節、人体入力、希望／採用形態、
余裕、障害物距離、flight state・gate・failsafe等を同じ時刻へ同期する。
3D、時系列、イベントを連動させ、再生・停止・シーク・速度変更・表示topic選択を設ける。

指定範囲のMP4/GIFと現在フレームのPNG/JPEGはブラウザ内で出力する設計とし、クラウド保存・共有URL管理は含めない。
大容量bagはROS側・ローカルで必要topicを抽出して軽量化し、再生データをロボットPCへ保存する。
変換後は元bagを削除することを既定とし、保持・サイズ・サンプリング上限をROS側のlaunch引数で設定できるようにする。
非公開topic、個人情報、映像・位置情報は利用者が出力前に確認する。

## 参考文献

[1] M. Zhao, T. Anzai, F. Shi, X. Chen, K. Okada, and M. Inaba,
“Design, Modeling, and Control of an Aerial Robot DRAGON: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation,”
*IEEE Robotics and Automation Letters*, vol. 3, no. 2, pp. 1176–1183, 2018.
DOI: 10.1109/LRA.2018.2793344.

[2] T. Anzai, M. Zhao, M. Murooka, F. Shi, K. Okada, and M. Inaba,
“Design, Modeling and Control of Fully Actuated 2D Transformable Aerial Robot with 1 DoF Thrust Vectorable Link Module,”
*2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2820–2826, 2019.

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

[7] C. Pacchierotti and D. Prattichizzo,
“Cutaneous/Tactile Haptic Feedback in Robotic Teleoperation: Motivation, Survey, and Perspectives,”
*IEEE Transactions on Robotics*, vol. 40, pp. 978–998, 2024.
DOI: 10.1109/TRO.2023.3344027.
