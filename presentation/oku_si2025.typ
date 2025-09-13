#import "./typst-jp-conf-template/jaconf/lib.typ": jaconf, definition, lemma, theorem, corollary, proof, appendix
#import "@preview/roremu:0.1.0": roremu
#import "@preview/equate:0.3.2": equate

#let abstract = [
  #lorem(80)
]

// デフォルト値でよい引数は省略可能
#show: jaconf.with(
  // 基本 Basic
  title: [6自由度の力覚フィードバックを有する \ 空中マニピュレーションのための遠隔操作システム],
  title-en: [Teleoperation system with 6-DoF haptic feedback for aerial manipulation],
  authors: [◯ 奥　朋哉 (東京大学)，金子　輝太朗 (東京大学)，趙　漠居 (東京大学)],
  authors-en: [◯ Tomoya OKU (The University of Tokyo)，Kotaro KANEKO (The University of Tokyo)，\ Moju ZHAO (The University of Tokyo)],
  abstract: abstract,
  // フォント名 Font family
  font-heading: "Yu Gothic",  // サンセリフ体，ゴシック体などの指定を推奨
  font-main: "Yu Mincho",  // セリフ体，明朝体などの指定を推奨
  font-latin: "TeX Gyre Termes",
  font-math: "New Computer Modern Math",
  // 外観 Appearance
  paper-margin: (top: 20mm, bottom: 27mm, left: 15mm, right: 15mm),
  paper-columns: 2,  // 1: single column, 2: double column
  page-number: none,  // e.g. "1/1"
  column-gutter: 4%+0pt,
  spacing-heading: 1.2em,
  bibliography-style: "sice.csl",  // "sice.csl", "rsj.csl", "ieee", etc.
  abstract-language: "en",  // "ja" or "en"
  keywords-language: "en",  // "ja" or "en"
  front-matter-spacing: 1.5em,
  front-matter-margin: 2.0em,
  // 見出し Headings
  heading-abstract: [*Abstract : *],
  heading-keywords: none,
  heading-bibliography: [参考文献],
  heading-appendix: [付録],
  // フォントサイズ Font size
  font-size-title: 16pt,
  font-size-title-en: 16pt,
  font-size-authors: 12pt,
  font-size-authors-en: 12pt,
  font-size-abstract: 9pt,
  font-size-heading: 12pt,
  font-size-main: 10pt,
  font-size-bibliography: 10pt,
  // 補足語 Supplement
  supplement-image: [Fig.],
  supplement-table: [Table],
  supplement-separator: [~~],
  // 番号付け Numbering
  numbering-headings: "1.1. ",
  numbering-equation: "(1)",
  numbering-appendix: "A.1",  // #show: appendix.with(numbering-appendix: "A.1") の呼び出しにも同じ引数を与えてください．
)

#set math.equation(
  supplement: [式],                   // 参照の前に「式」を付ける
)

#show ref.where(form: "normal"): set ref(supplement: auto)
#show figure.caption: set text(size: 10pt) // figureのキャプションのフォントサイズを変更
#show: equate.with(number-mode: "line")

#let skew = $op("skew")$

= 緒言 <sec:introduction>
近年，空中マニピュレーション~(aerial manipulation)が注目を集めている．ドローンをはじめとする空中ロボット~(aerial robot) は高い機動性と広い作業空間を持ち，遠隔地や人間の立ち入りが困難な場所での作業が可能になる．また，高所や有害物質が漂う環境など，人間にとって危険な場所での接触作業も可能にする．作業はロボットが完全自律で行なうことが理想であり，自律制御の研究は大きく進展している@Shi2019 ．しかし，空中作業環境は複雑で外乱も多いため，人間による操縦は現在のところ必要不可欠である@Darvish2023．人間の判断能力を活用することで，未知の環境や作業対象に対してもリアルタイムに最適な対応が可能となる．この理由から，空中ロボットの遠隔操作~(teleoperation) の研究が進められてきた．

従来の研究の多くは，3つの並進運動自由度と1つの回転運動自由度の4自由度を持つ不足駆動型マルチロータを対象としており@Nourmohammadi2018 @Aggravi2021 @Yashin2019 @Kim2020，一般的なジョイスティック型デバイスで操作可能であった．

一方で，より複雑な空中作業のために，3つの並進運動自由度と3つの回転運動自由度の6自由度を持つ完全駆動型マルチロータの研究も進められており，この操作には新しいデバイスを必要とする．@Allenspach2022 では地面に固定されたロボットアームを操作デバイスとする方法が提案されている．このデバイスは6自由度の独立入力を可能とするが，基部が固定されているため移動が困難であるという欠点がある．一方，人間の手の位置と姿勢を感知して操作入力を得る浮遊型操作デバイス@Macchini2020 が提案されている．この方法ではデバイスの一部を固定する必要がないため，操作者の手の動きを妨げず，完全駆動型空中ロボットの遠隔操作に適している．

遠隔操作ではロボットから操縦者へのフィードバックが不可欠であり，特に外部環境との接触を伴う作業では円滑な操作のために力覚フィードバックが重要である．多くの研究で用いられる力覚フィードバックは振動など単純なものが多い．しかし，作業を円滑に行うためには，デバイスに加わる力を力として表現することが望ましい．例えば，6つのプロペラを用いてハンドルに3次元力を加えるデバイス@Heo2018 や，長い棒の両端に2つのクアッドロータを取り付けることで，1方向の力と2方向のトルクを提供するデバイス@Sasaki2018 が提案されている．これらの研究は，基部を固定せずに操作者に力とトルクを提示することに成功しているが，6次元の力とトルクを独立して提示することはできない．したがって，本研究では，3次元の力と3次元のトルクで垂直に配置された6次元ベクトルである全レンチを独立して提示できる浮遊型デバイスを提案する．遠隔操作システムの全体図を@fig:system に示す．本研究の貢献は以下の通りである．

- 操縦者に6次元のレンチをすべて独立して提示できる浮遊型遠隔操作デバイスを提案する．
- 完全駆動型空中ロボットの全自由度を同時に制御可能な6次元レンチフィードバックを備えた遠隔操作システムを提案する．これにより精密かつ長距離操作を実現する．
- 障害物回避や傾斜壁の清掃を含む実験を通じて，提案システムの有効性を検証した．

#figure(
  image("figures/system.png", width: 70%),
  caption: [
    Diagram of our proposed teleoperation system. $bold(p)_"target"$ and $bold(q)_"target"$ are target position and quaternion of the aerial robot, $bold(f)_"feedback"$ and $bold(tau)_"feedback"$ are feedback forces and torques.
  ]
)<fig:system>

= Twin-Hammer
== デザイン
本研究で用いた遠隔操作デバイス「Twin-Hammer」は空中ロボットのタスク実行を容易に実行することを可能にする．人間が行なう多くのタスクは様々なツールを必要とし，これらのツールの操作を通して外界と相互作用し，反力に関する情報を得る．したがって，遠隔操作によりタスクを実行する場合には，ロボットのツールが操縦者の手元に物理的に存在しているかのようにデバイスを握り，操縦できることが直観的である．@sec:introduction で述べたように，空中ロボットに対する遠隔操作では，デバイスは空中に浮遊しており，6次元のレンチを独立に提示できる必要がある．これらの機能を実現するために，我々は推力と推力偏向機能を用いた．

推力偏向機能はZhaoら@Zhao2018 によって提示されたものの修正版である．@fig:hardware の右側に示すように，この推力偏向機能は2方向に回転させることができる．両端部に推力を生むロータを2つ取り付けることにより，合計で4自由度を持つ．ツールのように設計された長い棒の両端部にこのモジュールを取り付けることで，全体としてこのデバイスは8自由度の操作自由度を得る．この自由度は我々が望むレンチの6自由度よりも多いため，すべてのレンチを冗長性を持って扱うことを可能にする．

#figure(
  image("figures/hardware.png", width: 100%),
  caption: [
    Hardware design of proposed teleoperation device. Left: Whole view. Right: Thrust vectoring mechanism. The red arrows mean the thrust, and blue arrows is the rotational direction.
  ]
)<fig:hardware>

== 制御
本デバイスの制御自由度は@fig:device の上側に示す通りであるが，下側に示す仮想推力で考える．まず，仮想推力ベクトルを次のように定義する．
$
  bold(f)_("v"1) &= display(mat(f_("v"1 x), f_("v"1 y), f_("v"1 z))^top) \
  bold(f)_("v"2) &= display(mat(f_("v"2 x), f_("v"2 y), f_("v"2 z))^top) \
  bold(f)_"v" &= display(mat( f_("v"1 x), f_("v"1 y), f_("v"1 z), f_("v"2 x), f_("v"2 y), f_("v"2 z) ) ^ top)
$

仮想推力$bold(f)_"v"$から所望のトルクへの割当行列~(allocation matrix) を@eq:virtual_thrust_matrix のように表せる．
$
  Q_1 = mat(
    I_3, I_3 ;
    skew(bold(p)_"v1"), skew(bold(p)_"v2");
  )
$<eq:virtual_thrust_matrix> 
ただし，$I_3 in RR^(3 times 3)$は単位行列，$bold(p)_("v"i) in RR^3$ は仮想推力$bold(f)_("v"i) in RR^3$の点の位置ベクトル，$skew(bold(p)) in RR^(3 times 3)$ は $bold(p) in RR^3$の交代行列であり，@eq:skew のように外積を行列で表現できる．
$
  bold(a) times bold(b) = skew(bold(a)) bold(b)
$<eq:skew>
このとき，所望のレンチである@eq:des_wrench を用いて@eq:allocation のように書ける．
$
  bold(W)_"des" = mat(
    F_("des"x), F_("des"y), F_("des"z), T_("des"x), T_("des"y), T_("des"z),  
  )^top
$<eq:des_wrench>
$
  Q_1 bold(f)_"v" = bold(W)_"des" + bold(g)
$<eq:allocation>
ここで，$bold(g) = display(mat(0, 0, M g, 0, 0, 0)^top) $は重力項である．ただし，$M$はデバイスの質量であり，$g$は重力加速度である．


$x$軸まわりのトルクを除いた所望のレンチである@eq:des_wrench_except_x を用いて，@eq:allocation_except_x のように書ける．
$
  bold(W)'_"des" = mat(
    F_("des"x), F_("des"y), F_("des"z), T_("des"y), T_("des"z),  
  )^top
$<eq:des_wrench_except_x>
$
  Q'_1 bold(f)_"v" = bold(W)'_"des" + bold(g)'
$<eq:allocation_except_x>
ここで，$bold(g)' = display(mat(0, 0, M g, 0, 0)^top) $は重力項である．また，仮想推力$bold(f)_"v"$ は@eq:virtual_thrust のように求められる．
$
  bold(f)_"v" = Q'_1^"#" (bold(W)'_"des" + bold(g)')
$<eq:virtual_thrust>
また，$Q'_1^"#" in RR^(6 times 5)$は#text(fill: red)[$x$まわりのトルクと一致する第4成分を持った]$Q'_1$の疑似逆行列である．

また，ベクトル角は以下のように計算される．#text(fill: red)[ここのマイナスはなぜ？→$theta$の決め方]
$
  theta_i = arctan(-f_("v"i y) / f_("v"i z)), \
  phi_i = arctan(f_("v"i x) / (-f_("v"i y) sin theta_i +  f_("v"i z) cos theta_i ))
$

$x$軸回りに生じるcounterトルクは次のように表される．
ここで，$I_i$は$x$軸回りの慣性モーメントである．
$
  tau_"counter" = sum I_i dot.double(theta_i)
$
#text(fill:red)[仮想推力と$x$軸回りのトルクから実際の推力への割当行列は以下のように表現される．]
$
  Q_2 = mat(
    1, 0, 1, 0;
    0, 1, 0, 1;
    d_1 cos phi_1, d_2 cos phi_1, -d_3 cos phi_2, -d_4 cos phi_2;
  )
$
ここで，$d_1, d_2, d_3, d_4$は各ローターからデバイスの縦方向までの長さである．

また，実際の推力$bold(f)=display(mat(f_1, f_2, f_3, f_4)^top) $は以下のように計算される．
$
  bold(f) = Q_2^"#" mat( abs(bold(f)_("v"1)); abs(bold(f)_("v"1)); T_("des"x)+tau_"counter")
$

ここで，$Q_2^"#" in RR^(4 times 3)$は$Q_2$の疑似逆行列である．

ローターによって生じる力の範囲には限りがあるため，ここで得られた計算は常に実行可能である訳ではない．
もし，実行可能範囲の外にある場合，実行可能な値の中で最も近い値で再計算することとなる．

#figure(
  image("figures/device.png", width: 75%),
  caption: [Top: Control DoF of the proposed device. The black allows shows the hadle coordinate system fixed on the device. Bottom: Virtual thrust considered on a control model.]
)<fig:device>

= システム
本章では，完全駆動型空中ロボットを用いて作業を実行する遠隔操作システムについて述べる．このシステムは，操縦者からロボットへの位置指令の生成と，ロボットから操縦者への力フィードバックの生成から構成される．

== 位置操縦コマンド

空中ロボットの位置操作については主に2つの機能が要求される．1つは長距離の移動であり，実質的に無限の広さを持つ空間を移動するために高速な移動が必要である．もう1つは精密な操作であり，正確に細かい作業を行うための機能である．これら2つの機能を達成するため，我々は位置マッピングモードと速度マッピングモードを提案した．

=== 位置マッピングモード
位置マッピングモードは正確なタスク実行のために用いる．このモードにおいて目標位置$bold(p)_"target"$と目標四元数$bold(q)_"target"$は以下のように計算される．
$
  bold(p)_"target" &= bold(p)_"robot" (t_0) + bold(k)_1 dot.circle (bold(p)_"device" (t) - bold(p)_"device" (t_0)) \
  bold(q)_"target" &= bold(q)_"device" (t)
$
ここで，$t_0$はこのモードに入った初期時刻，$dot.circle$はHadamard積であり，ベクトルの各成分の積を表す．$bold(k)_1$はタスクのタイプに応じて調整されるスケーリングパラメータである．

=== 速度マッピングモード
一方，速度マッピングモードは広域移動のために用いる．このモードにおいて目標位置$bold(p)_"target"$と目標四元数$bold(q)_"target"$は以下のように計算される．
$
  bold(p)_"target" &= bold(p)_"robot" (t) + bold(k)_2 dot.circle (bold(p)_"device" (t) - bold(p)_"device" (t_0)) \
  bold(q)_"target" &= bold(q)_"robot" (t) + bold(k)_3 dot.circle (bold(q)_"device" (t) - bold(q)_"device" (t_0))
$
ここで，$bold(k)_2, bold(k)_3$はスケーリングパラメータである．各時刻においてロボットの位置と四元数にデバイスの変位量を加えることはロボットの速度を変化させることに等しい．

このモードでは，操作者が装置の原点を認識することが重要であり，ジョイスティック操作時にも同様である．本研究で提案する装置は，装置の原点からの移動量に対応する反力を提供することで，操作者が原点を認識できるようにする．コマンドや制御と同様に，フィードバック力の大きさを装置の移動量に比例させると，原点付近の力が小さくなり，原点を視認しにくくなる．そこで，以下のように対数変換を用いる．
$
  bold(f)_"feedback" &= log [ bold(k)_4 dot.circle (bold(p)_"device" (t) - bold(p)_"device" (t_0)) ] \
  bold(tau)_"feedback" &= log [ bold(k)_5 dot.circle (bold(q)_"device" (t) - bold(q)_"device" (t_0)) ]
$
ここで，$bold(k)_4, bold(k)_5$はスケーリングパラメータである．この変換により原点付近のフィードバック力が大きくなり，原点の認識が容易になる．さらに操作の難易度を低減するため，原点付近にはロボットに指令を送らない停止ゾーンを設定している．これら二つのモードを適切に活用することで，広い作業空間において迅速かつ精密な作業が可能となる．

== 力とトルクのフィードバック
空中ロボットの作業用エンドエフェクタには6自由度の力とトルクが作用する．作業を円滑に遂行するためには，操作者がこれら6次元の力とトルクを正確に認識しなければならない．Stevensら@Stevens1960 の研究により，人間の知覚における変化の大きさと実際の物理的刺激の大きさの間には，以下に示すような関係があることが示されている．
$
  Phi (I) = k I^alpha
$
ここで，$Phi (I)$は知覚される刺激の大きさ，$I$は実際の物理的刺激の大きさ，$k$は比例定数，$alpha$は刺激の種類に依存する指数である．腕全体で重力を認識する際の$alpha$は$1.45$であるとされる．本研究で提案した装置は操作者の腕全体に力覚フィードバックを提供するため，物理的刺激は同一であるとみなす．

操作者がレンチの変化を正しく認識できるようにするためには，この指数関数的な関係を打ち消し，知覚を直線的にすることが重要である．したがって，操作者に提示されるレンチは，ロボットマニピュレータに適用されるレンチから以下のように生成される．
$
  bold(w)_"feedback" = log_alpha (bold(k)_7 dot.circle bold(w)_"measured")
$
ここで，$bold(w)_"feedback"$は操縦者にフィードバックされるレンチであり，$bold(w)_"measured"$はロボットのエンドエフェクタで計測されるレンチである．$bold(k)_7$は大きさを調節するパラメータである．

= 実験
#roremu(1024)

= 結言
#roremu(1024)

#bibliography(
  "references.bib",
)