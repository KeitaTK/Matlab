# rls_joint_estimation_sim

目的
- RLS を拡張して、周期外力モデルの係数 A, B, C だけでなく、**角周波数 (omega)** も同時にオンライン推定できるかを検証するシミュレーション。

フォルダ構成
- scripts/
  - `generate_damped_sine.m` (既存のものを利用可) — 減衰正弦信号を生成して `data/` に保存
  - `rls_joint_estimator.m` — A,B,C と omega を同時に推定する実装（アルゴリズム案付き）
  - `run_sim_joint.m` — 実験を実行して `results/` に保存するランナー
  - `plot_results_joint.m` — 推定結果と真値、パラメータ推移をプロットして保存（ファイル名に設定値を含める）
  - `sweep_params_joint.m` — lambda, noise, initial omega などをスイープして評価
- data/ — 生成された入力 CSV を保存
- results/ — 出力 CSV, MAT, PNG を保存

実験設計（短く）
- サンプリング周波数: 100 Hz
- 信号: 減衰正弦 f_true = 0.8 Hz（複数の減衰率で実験）
- RLS 方針: 入力ベクトルは [sin(phase), cos(phase), 1, (optionally) d(sin)/dω 項] 等を検討
- 目標: 初期 ω を誤差（例: 0.6 Hz）で開始し、オンラインで true freq に収束するかを評価

評価指標
- 収束時間: |est_freq - true_freq| < 2% を満たす最初の時刻
- 定常誤差: 最後 10 s の平均誤差
- 推定分散: 最後 10 s の標準偏差

次の実装タスク（提案）
1. `rls_joint_estimator.m` のプロトタイプ実装（数値安定化・ω 更新の平滑化・閾値ガードを含む）
2. `run_sim_joint.m` を作成して基本実験を1回実行
3. `sweep_params_joint.m` でパラメータ感度を評価

備考
- 現在の実装（`rls_phase_estimator.m`）をベースに、ω を状態として追加するか、θ の位相傾きをより直接的に ω にマップする手法を試すことを推奨します。

---

作業開始して良いですか？（実装に進めます）