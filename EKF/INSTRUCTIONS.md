# 実装インストラクション（正弦波モデルEKF）

## 目標
RLSの思想（正弦波 + DCオフセット）を継承しつつ、EKFでオンライン周波数推定を可能にする。

## 本実装で採用するモデル
- 状態: `x = [d, d_dot, c, omega]^T`
  - `d`: 周期外乱の正弦波成分
  - `d_dot`: その時間微分
  - `c`: DCオフセット
  - `omega`: 角周波数 [rad/s]
- 状態方程式（連続系）:
  - `d_dot = x2`
  - `x2_dot = -omega^2 * d`
  - `c_dot = 0`
  - `omega_dot = 0 + process_noise`
- 観測方程式:
  - `y = d + c + v`

## 必須要件
- オンラインで以下を推定すること:
  - 揺れ状態（`d`, `d_dot`）
  - オフセット（`c`）
  - 周波数（`omega`, `f=omega/(2*pi)`）
- 以下の条件で検証すること:
  - ノイズあり
  - 初期周波数ミスマッチ
  - 高周波干渉あり

## 実装済みスクリプト
- `scripts/ekf_harmonic_step.m`
- `scripts/generate_signal_baseline.m`
- `scripts/generate_signal_noisy.m`
- `scripts/generate_signal_highfreq_interference.m`
- `scripts/run_ekf_case.m`
- `scripts/run_ekf_sweep.m`
- `scripts/evaluate_ekf_metrics.m`
- `scripts/plot_ekf_results.m`
- `scripts/run_demo_harmonic_ekf.m`

## 出力ルール
- 入力信号: `data/`
- 実験結果: `results/`
- 実験サマリ: `results/` 配下のCSVまたはMarkdown

## 評価指標
- 2%誤差帯への収束時間
- 最後10秒の定常バイアス
- 最後10秒の定常標準偏差
- 再構成信号のNRMSE
- イノベーション平均・ラグ1相関

## チューニング手順
1. ノイズなしベースラインを実行
2. `R` を固定して `Q`（特に `q_omega`）を調整
3. ノイズありケースで `R` を再調整
4. 高周波干渉ケースで `q_omega` と `q_d_dot` を再調整
5. `configs/experiment_matrix.md` に従って一括評価

## 更新ルール
新しい実験を追加したら、以下を更新すること:
- `docs/03_validation_plan.md`
- `results/` のサマリ資料

## 利用ライブラリ（MATLAB Toolbox）
本プロジェクトで確認済みの主要Toolbox:
- MATLAB
- Signal Processing Toolbox
- Control System Toolbox
- Statistics and Machine Learning Toolbox
- System Identification Toolbox
- DSP System Toolbox

備考:
- 実験で追加Toolboxを使った場合は、この節に追記すること。
