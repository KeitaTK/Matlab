# 現行コードベース調査（RLSからEKFへの移行）

## 調査対象
`Drone-simulation` 配下のRLSベース推定資産を確認し、EKF移行に必要な要件を抽出した。

## 既存RLS資産

### 1) `Drone-simulation/rls_phase_estimation_sim`
- 手法: `A, B, C` をRLS推定し、位相バッファ勾配から周波数更新
- 主要スクリプト: `scripts/rls_phase_estimator.m`
- 信号生成:
  - `scripts/generate_damped_sine.m`
  - `scripts/generate_multi_frequency_sine.m`
- 特徴:
  - 勾配学習率が不要
  - ノイズ耐性が高い
  - 実装が比較的シンプル

### 2) `Drone-simulation/rls_joint_estimation_sim`
- 手法: 交互RLS（`A, B, C`）+ 勾配法（`omega`）
- 主要スクリプト: `scripts/rls_joint_estimator.m`
- 高周波干渉試験スクリプトあり
- 特徴:
  - 初期周波数ズレからの収束が速い
  - 同時推定の実績あり

## 共通信号モデル
既存研究で共通に扱っている観測モデル:

$$
 y_k = A \sin(\omega t_k) + B \cos(\omega t_k) + C + n_k
$$

この形が、正弦波モデルEKFへの橋渡しになる。

## 継承すべき検証テーマ
- 初期周波数ミスマッチ
- 加法性白色ノイズ耐性
- 減衰包絡付き信号
- 高周波干渉（外乱重畳）

## RLSから見た課題
- 非線形パラメータ（周波数）の不確かさ伝播が限定的
- イノベーション整合性を含むEKF標準診断が使いにくい
- モデル拡張時に共分散整合を取りにくい

## EKF移行要件
- 状態と周波数の同時推定
- 非線形モデルの共分散伝播
- 既存RLSと同一条件で比較可能な実験計画
- 収束時間・バイアス・分散・再構成誤差で比較
