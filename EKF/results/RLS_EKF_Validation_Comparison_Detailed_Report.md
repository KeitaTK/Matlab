# 生成検証信号による RLS と EKF の性能比較レポート

- 生成日時: 2026-04-01 22:47:15
- MATLAB: 25.2.0.2998904 (R2025b)

## 1. 検証条件
- 信号はすべてコード内で生成（ログ再利用ではない）
- ケース: baseline / noisy_010 / interf_5hz_r05
- 評価指標: 収束時間, 定常バイアス, 定常標準偏差, NRMSE, イノベーション自己相関(1次)
- 同一定義の指標を EKF と RLS に適用して比較

## 2. 比較結果（数値）

| case_id | ekf_conv_s | rls_conv_s | ekf_bias_hz | rls_bias_hz | ekf_std_hz | rls_std_hz | ekf_nrmse | rls_nrmse |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | 0.3800 | NaN | 0.002607 | -0.072886 | 0.002388 | 0.002442 | 0.006987 | 0.220962 |
| noisy_010 | 0.5600 | NaN | 0.002639 | -0.072700 | 0.006076 | 0.002466 | 0.041128 | 0.172729 |
| interf_5hz_r05 | 2.3700 | NaN | 0.004795 | -0.074633 | 0.033616 | 0.002525 | 0.117188 | 0.188674 |

## 3. デバッグ検証サマリ（末尾10秒）

| case_id | ekf_nonfinite | rls_nonfinite | ekf_last10_mean_hz | rls_last10_mean_hz | ekf_last10_std_hz | rls_last10_std_hz | abs_err_gap(ekf-rls)_hz |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 0 | 0 | 0.802607 | 0.727114 | 0.002388 | 0.002442 | -0.070059 |
| noisy_010 | 0 | 0 | 0.802639 | 0.727300 | 0.006076 | 0.002466 | -0.067511 |
| interf_5hz_r05 | 0 | 0 | 0.804795 | 0.725367 | 0.033616 | 0.002525 | -0.046739 |

## 4. ケース別サマリ（どちらが優位か）

| case_id | abs_bias_small | std_small | nrmse_small |
|---|---|---|---|
| baseline | EKF | EKF | EKF |
| noisy_010 | EKF | RLS | EKF |
| interf_5hz_r05 | EKF | RLS | EKF |

## 5. デバッグ観点での検証結果

- 両推定器で非有限値（NaN/Inf）が混入しないかをケースごとに確認
- 末尾10秒の周波数推定平均/分散を標準出力で確認
- 指標差分（EKF-RLS）平均:
  - |定常バイアス|差: -0.070059 Hz
  - 定常標準偏差差: 0.011549 Hz
  - NRMSE差: -0.139021

## 6. 結果の考察

1. 今回の3ケースでは、|定常バイアス|とNRMSEは全ケースでEKFが優位。RLSは推定値が0.73Hz付近までしか上がらず、真値0.8Hzに対して恒常的な負バイアスが残った。
2. 一方で定常標準偏差は、baselineでは両者ほぼ同等、noisy/interferenceではRLSが小さく、RLSは"滑らかだが偏る"挙動、EKFは"真値追従性は高いが揺れやすい"挙動を示した。
3. interferenceではEKFの標準偏差が増大（高周波干渉を単一周波数モデルで吸収しきれないため）。ただしNRMSEはEKFの方が小さく、平均的な出力再現性は維持された。
4. rls_convergence_time_s がNaNなのは、2%収束判定帯に60秒内で入らないことを示し、設定（phase_update_interval, omega_alpha, lambda）再調整の余地を示唆する。
5. 次の改善方針として、EKFは2成分モデル化（対象周波数+干渉周波数）を優先、RLSは位相更新を高頻度化して定常バイアスの解消を優先するのが妥当。

## 7. 出力ファイル
- rls_ekf_case_comparison.csv
- rls_ekf_debug_summary.csv
- rls_ekf_case_comparison.mat
- rls_ekf_case_comparison.png
- RLS_EKF_Validation_Comparison_Detailed_Report.md
