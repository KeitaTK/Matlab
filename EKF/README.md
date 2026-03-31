# EKF 周波数・揺れ推定ワークスペース

## 目的
このディレクトリは、既存のRLSベース推定を正弦波モデルEKFへ置き換えるための実験・実装拠点です。

本ワークスペースでは、以下を同時に推定します。
- 揺れの正弦波成分（状態）
- DCオフセット
- 角周波数（およびHz表記の周波数）

## 現在の状態
- ドキュメント整備: 完了
- 正弦波モデルEKF実装: 初版を `scripts/` に実装済み
- 追加チューニングと比較評価: これから実施

## ディレクトリ構成
- `docs/`: 調査、数式仕様、検証計画、実験シナリオ
- `configs/`: 実験マトリクス定義
- `scripts/`: MATLAB実装（EKF本体、信号生成、評価、可視化）
- `data/`: 生成信号データ
- `results/`: 実験出力（MAT/CSV/PNG）
- `EKF.m`: 既存の別系統EKF資産（そのまま保持）

## 最初に読むファイル
1. `docs/01_current_codebase_survey.md`
2. `docs/02_ekf_design_math.md`
3. `docs/03_validation_plan.md`
4. `INSTRUCTIONS.md`

## すぐ試す
MATLABで `EKF/scripts/run_demo_harmonic_ekf.m` を実行すると、
高周波干渉ケースでのEKF推定デモが `results/` に保存されます。
