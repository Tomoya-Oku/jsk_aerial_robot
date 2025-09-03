#import "@preview/jaconf:0.5.0": jaconf, definition, lemma, theorem, corollary, proof, appendix
#import "@preview/roremu:0.1.0": roremu

// デフォルト値でよい引数は省略可能
#show: jaconf.with(
  // 基本 Basic
  title: [空中マニピュレーションのための6自由度力覚フィードバックを用いたテレオペレーションシステム],
  title-en: [Teleoperation system with 6-DoF haptic feedback for aerial manipulation],
  authors: [◯ 奥　朋哉 (東京大学)，金子　輝太朗 (東京大学)，趙　漠居 (東京大学)],
  authors-en: [◯ Tomoya OKU (The University of Tokyo)，Kotaro KANEKO (The University of Tokyo)，\ Moju ZHAO (The University of Tokyo)],
  abstract: [#lorem(80)],
  // フォント名 Font family
  font-heading: "IPAGothic",  // サンセリフ体，ゴシック体などの指定を推奨
  font-main: "IPAMincho",  // セリフ体，明朝体などの指定を推奨
  font-latin: "Times New Roman",
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
  heading-abstract: [*Abstract--*],
  heading-keywords: none,
  heading-bibliography: [参考文献],
  heading-appendix: [付録],
  // フォントサイズ Font size
  font-size-title: 16pt,
  font-size-title-en: 12pt,
  font-size-authors: 12pt,
  font-size-authors-en: 12pt,
  font-size-abstract: 10pt,
  font-size-heading: 12pt,
  font-size-main: 11pt,
  font-size-bibliography: 9pt,
  // 補足語 Supplement
  supplement-image: [Fig.],
  supplement-table: [Table],
  supplement-separator: [ ],
  // 番号付け Numbering
  numbering-headings: "1.1. ",
  numbering-equation: "(1)",
  numbering-appendix: "A.1",  // #show: appendix.with(numbering-appendix: "A.1") の呼び出しにも同じ引数を与えてください．
)

#let skew(arg) = $ s k e w (arg) $

= 緒言
近年，空中マニピュレーション (aerial manipulation) が注目を集めている．空中ロボットは高い機動性と広い作業空間を持ち，遠隔地や人間が立ち入ることの難しい場所で作業を可能にする．また高所や有害物質が浮遊する環境など，人間にとって危険な場所での接触作業も可能にする．理想的にはロボットが完全自律で作業することだが，自律制御の研究は大きく進展しているものの，空中作業環境は複雑で外乱も多いため，人間のオペレータをループに含める必要がある．人間の判断能力を利用することで，未知の環境や未知の対象物に対してもリアルタイムに最適な対応が可能となる．この理由から，空中ロボットの遠隔操作 (teleoperation) の研究が進められてきた．

従来の研究の多くはアンダーアクチュエート型マルチロータ (4自由度: 3平行移動+1回転) を対象としており，一般的なジョイスティック型デバイスで操作可能であった．一方で，より複雑な空中作業のためにフルアクチュエート型マルチロータ (6自由度：3平行移動+3回転) の研究も進められている．これらは新しい操作デバイスを必要とする．既存研究では，地面に固定されたロボットアームを操作デバイスとする方法や，手の位置・姿勢を検出するフローティング型デバイスが提案されている．後者はオペレータの手の動きを妨げず，フルアクチュエート型ロボットの遠隔操作に適している．

遠隔操作では，ロボットからオペレータへのフィードバックが不可欠である．特に環境との接触を伴う作業では力覚フィードバックが重要である．従来研究では振動などの単純なフィードバックが多いが，作業を円滑に行うには，ツールに加わる力を実際の力として提示することが望ましい．過去にはプロペラによる推力で3次元の力を提示する装置や，棒の両端にクアッドロータを取り付けて1方向の力と2方向のトルクを提示する装置が提案されたが，いずれも6次元の力・トルクを独立して提示することはできなかった．

そこで本研究では，3次元の力と3次元のトルクを独立に提示できるフローティング型デバイスを提案する．これを用いた遠隔操作システムにより，フルアクチュエート型空中ロボットの全自由度を直感的に操作でき，さらに6次元の力覚フィードバックを得ながら精密作業と広域移動を両立できることを示す．

= デバイス
== デザイン


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

仮想推力$bold(f)_"v" = mat( bold(f)_"v1", bold(f)_"v2" ; ) ^ top$ は$x$軸まわりのトルクを除いた所望のレンチ
$
  bold(W)_"des" = mat(
    F_("des"x), F_("des"y), F_("des"z), F_("des"y), F_("des"z),  
  )^top
$
から@eq:virtual_thrust のように求められる．
$
  bold(f)_"v" = Q_1^"#" (bold(W)_"des" + bold(g))
$<eq:virtual_thrust>
ただし，$bold(g) = mat(0, 0, M g, 0, 0, 0)^top$は重力項である．ただし，$M$はデバイスの質量であり，$g$は重力加速度である．また，$Q_1^"#" in RR^(6 times 5)$は$x$まわりのトルクと一致する第4成分を持った$Q_1$のpseudo-inverseである．

また，ベクトル角は以下のように計算される．
$
  theta_i = arctan(-f_("v"i y) / -f_("v"i z)), \
  phi_i = arctan(f_("v"i x) / (-f_("v"i y) sin(theta_i) +  f_("v"i z) cos(theta_i) ))
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
#roremu(1024)

= 実験
#roremu(1024)

= 結言
#roremu(1024)

#bibliography(
  "references.bib",
)