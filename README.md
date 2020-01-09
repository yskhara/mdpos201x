# 概要
　ブラシDCモータドライバmdpos201xのファームウェアです．速度制御または位置制御を行うことができます．
　STマイクロエレクトロニクス社のLL/HALライブラリなどのIPを含みます．

# 動作確認
  releasesから.elfファイルを落としてくる
  CubeProgrammerで書き込む

  UARTで繋いでパラメータを設定する(以下は775,24:1の場合)
  ```
  SKPR 0.5
  SKIT 20
  SKEM 0.14
  SKGT 0.69
  SPPR 2000
  SKRF 1.0
  SMVL 100
  SHVL 1.0
  SMTQ 8.0
  SVSP 20
  SKVP 40
  ```

  mdに安定化電源,ESを繋ぐ

  UARTで
  ```
  SENV 1
  ```
  でenableしたあと

  ```
  SVTG 50
  ```

  で速度指令ができる.
