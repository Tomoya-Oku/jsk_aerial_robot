#import "@preview/jaconf:0.5.0": jaconf, definition, lemma, theorem, corollary, proof, appendix
#import "@preview/roremu:0.1.0": roremu

#let abstract = [
  #lorem(80)
]

// デフォルト値でよい引数は省略可能
#show: jaconf.with(
  // 基本 Basic
  title: [6自由度の力覚フィードバック搭載デバイスを用いた \ 空中マニピュレーションのためのジェスチャー認識 (仮)],
  title-en: [Gesture Recognition system with the 6-DoF haptic feedback device \ for aerial manipulation],
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
  bibliography-style: "ieee",  // "sice.csl", "rsj.csl", "ieee", etc.
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

#show figure.caption: set text(size: 10pt) // figureのキャプションのフォントサイズを変更

#let skew(arg) = $ s k e w (arg) $

= 緒言 <sec:introduction>
近年，空中マニピュレーション (aerial manipulation) が注目を集めている．ドローンをはじめとする空中ロボット (aerial robot) は高い機動性と広い作業空間を持ち，遠隔地に加えて高所や有害物質が浮遊する環境など，人間にとって危険な場所での接触作業を可能にする．そこで，ロボットが完全自律で作業することが理想であるが，自律制御の研究は大きく進展しているものの，空中作業環境は複雑で外乱も多いため，人間による操縦は現在のところ必要不可欠である．人間の判断能力を利用することで，未知の環境や対象物に対してもリアルタイムに最適な対応が可能となる．この理由から，空中ロボットの遠隔操作 (teleoperation) の研究が進められてきた．

従来の研究の多くはアンダーアクチュエート型マルチロータ (4 DoF: 3 平行移動+1 回転) を対象としており，一般的なジョイスティック型デバイスで操作可能であった．一方で，より複雑な空中作業のためにフルアクチュエート型マルチロータ (6 DoF: 3 平行移動+3 回転) の研究も進められている．これらは新しい操作デバイスを必要とし，既存研究では地面に固定されたロボットアームを操作デバイスとする方法@Allenspach2022 や，手の位置・姿勢を検出するフローティング型デバイスが提案されている．後者は操縦者の手の動きを妨げず，フルアクチュエート型ロボットの遠隔操作に適している．

遠隔操作ではロボットからオペレータへのフィードバックが不可欠であり，特に環境との接触を伴う作業では力覚フィードバックが重要である．従来の研究では6次元の力・トルクを独立して提示することはできなかったが，本研究で用いるフローティング型デバイスは次元の力と3次元のトルクを独立に提示できる．これにより，フルアクチュエート型空中ロボットの全自由度を直感的に操作でき，さらに6次元の力覚フィードバックを得ながら精密作業と広域移動を両立できる．

しかし，現在は空中ロボットの飛行までの起動シーケンスおよび飛行中のモード切り替えをTwin-Hammer単体で行うことはできない．Twin-Hammerに対して複数のボタンを取り付けるなどの方法も考えられるが，

そこで本研究では，このデバイスのさらなる機能として，デバイスを用いたジェスチャーによるコマンドを用いて空中ロボットのアーミング，モード切り替えなどの機能を可能にし，
// TO-DO: 評価できるようなタスク

= Twin-Hammer
== デザイン
本研究で用いた遠隔操作デバイスは空中ロボットのタスク実行を容易に実行することを可能にする．人間が行なう多くのタスクは様々なツールを必要とし，これらのツールの操作を通して外界と相互作用，そして反力に関する情報を獲得する．したがって，遠隔操作によりタスクを実行する場合には，ロボットのツールが操縦者の手元に物理的に存在しているかのようにデバイスを握り，操縦できることが直観的である．@sec:introduction で述べたように，空中ロボットに対する遠隔操作では，デバイスは空中に浮遊しており，6次元のレンチを独立に提示できる必要がある．これらの機能を実現するために，我々は推力と推力偏向機能を用いた．

推力偏向機能はZhaoらによって提示されたものの修正版である@Zhao2018 ．@fig:hardware の右側に示すように，この推力偏向機能は2方向に回転させることができる．両端部に推力を生むロータを2つ取り付けることにより，合計で4自由度を持つ．ツールのように設計された長い棒の両端部にこのモジュールを取り付けることで，全体としてこのデバイスは8自由度の操作自由度を獲得する．この自由度は我々が望むレンチの6自由度よりも多いため，すべてのレンチを冗長性を持って扱うことを可能にする．

#figure(
  image("figures/Hardware design of proposed teleoperation device.png", width: 100%),
  caption: [Hardware design of proposed teleoperation device. Left:Whole
view. Right:Thrust vectoring mechanism. The red arrows mean the thrust,
and blue arrows is the rotational direction.]
)<fig:hardware>

== 制御
以下は全てデバイスに固定された慣性系で考える．@fig:device に仮想推力を示す．
$
  Q_1 = mat(
    I_3, I_3 ;
    skew(bold(p)_"v1"), skew(bold(p)_"v2");
  )
$
ただし，$I_3 in RR^(3 times 3)$はidentity matrix，
$bold(p)_("v"i)$ は仮想推力$bold(f)_("v"i)$の点での位置ベクトル，
$skew(bold(p))$ は $bold(p)$のskew-symmetric matrixである．

仮想推力$bold(f)_"v" = mat( bold(f)_"v1", bold(f)_"v2" ; ) ^ top$ は$x$軸まわりのトルクを除いた所望のレンチ@fig:des_wrench から
$
  bold(W)_"des" = mat(
    F_("des"x), F_("des"y), F_("des"z), F_("des"y), F_("des"z),  
  )^top
$<fig:des_wrench>
から@eq:virtual_thrust のように求められる．
$
  bold(f)_"v" = Q_1^"#" (bold(W)_"des" + bold(g))
$<eq:virtual_thrust>
ただし，$bold(g) = display(mat(0, 0, M g, 0, 0, 0)^top) $は重力項である．ただし，$M$はデバイスの質量であり，$g$は重力加速度である．また，$Q_1^"#" in RR^(6 times 5)$は$x$まわりのトルクと一致する第4成分を持った$Q_1$のpseudo-inverseである．

また，ベクトル角は以下のように計算される．
$
  theta_i = arctan(-f_("v"i y) / f_("v"i z)), \
  phi_i = arctan(f_("v"i x) / (-f_("v"i y) sin theta_i +  f_("v"i z) cos theta_i ))
$

$x$軸回りに生じるcounterトルクは次のように表される．
ここで，$I_i$は$x$軸回りの慣性モーメントである．
$
  tau_"counter" = sum(I_i dot.double(theta_i))
$
仮想推力と$x$軸回りのトルクから実際の推力へのallocation-matrixは以下のように表現される．
$
  Q_2 = mat(
    1, 0, 1, 0;
    0, 1, 0, 1;
    d_1 cos phi_1, d_2 cos phi_2, -d_3 cos phi_1, -d_4 cos phi_2;
  )
$
ここで，$d_1, d_2, d_3, d_4$は各ローターからデバイスの縦方向までの長さである．

また，実際の推力$bold(f)=mat(f_1, f_2, f_3, f_4)^top$は以下のように計算される．
$
  bold(f) = Q_2^"#" mat( abs(bold(f)_("v"1)); abs(bold(f)_("v"1)); T_("des"x)+tau_"counter")
$

ここで，$Q_2^"#" in RR^(4 times 3)$は$Q_2$のpseuso-inverseである．

ローターによって生じる力の範囲には限りがあるため，ここで得られた計算は常に実行可能である訳ではない．
もし，実行可能範囲の外にある場合，実行可能な値の中で最も近い値で再計算することとなる．

#figure(
  image("figures/device.png", width: 75%),
  caption: [Device]
)<fig:device>

= システム
空中ロボットの位置操作については主に2つの機能が要求される．
1つは長距離の移動であり，実質的に無限の広さを持つ空間を移動するために高速な移動が必要である．
もう1つは精密な操作であり，正確に細かい作業を行うための機能である．
これら2つの機能を達成するため，我々は位置マッピングモードと速度マッピングモードを提案した．

現在，



= 実験
#roremu(1024)

= 結言
#roremu(1024)

#bibliography(
  "references.bib",
)