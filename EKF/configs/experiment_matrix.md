# 実験マトリクス定義

## 共通設定
- サンプリング周波数: 100 Hz
- 実験時間: 60 s
- ターゲット周波数: 0.8 Hz
- 初期推定周波数: 0.6 Hz
- 乱数シード: [1, 2, 3, 4, 5]

## EKFパラメータテンプレート
- 初期共分散 `P0`（対角）:
  - `p_d, p_d_dot, p_c, p_omega`
- プロセスノイズ `Q`（対角）:
  - `q_d, q_d_dot, q_c, q_omega`
- 観測ノイズ `R`:
  - `r_y`

## 初期スイープ案
- Qセット:
  - A（保守）: `q_omega` 小
  - B（標準）: バランス
  - C（俊敏）: `q_omega` 大
- Rセット:
  - 1（小）
  - 2（中）
  - 3（大）

組み合わせ:
- 3つのQセット × 3つのRセット × シナリオ群

## 集計必須列
- `scenario_id`
- `q_set`
- `r_set`
- `seed`
- `convergence_time_s`
- `steady_bias_hz`
- `steady_std_hz`
- `nrmse`
- `innovation_mean`
- `innovation_lag1_corr`
- `pass_fail_flag`
