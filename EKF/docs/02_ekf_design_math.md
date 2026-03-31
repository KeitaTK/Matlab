# 正弦波モデルEKFの数式仕様

## 1. 目的
周期外乱を「正弦波 + DCオフセット」として内部モデル化し、周波数を同時推定する。

## 2. 状態定義

$$
\mathbf{x}_k =
\begin{bmatrix}
 d_k \\
 \dot d_k \\
 c_k \\
 \omega_k
\end{bmatrix}
$$

- $d_k$: 正弦波外乱成分
- $\dot d_k$: その時間微分
- $c_k$: DCオフセット
- $\omega_k$: 角周波数 [rad/s]

周波数Hzは

$$
f_k = \frac{\omega_k}{2\pi}
$$

## 3. 状態方程式（連続系）

$$
\dot d = \dot d
$$
$$
\ddot d = -\omega^2 d
$$
$$
\dot c = 0
$$
$$
\dot \omega = 0 + w_\omega
$$

ここで $w_\omega$ は周波数状態のランダムウォーク項。

## 4. 離散化（Euler）
サンプリング周期 $\Delta t$ として:

$$
d_{k+1} = d_k + \Delta t\,\dot d_k
$$
$$
\dot d_{k+1} = \dot d_k - \Delta t\,\omega_k^2 d_k
$$
$$
c_{k+1} = c_k
$$
$$
\omega_{k+1} = \omega_k
$$

プロセスノイズを加えて:

$$
\mathbf{x}_{k+1} = f(\mathbf{x}_k) + \mathbf{w}_k,\quad \mathbf{w}_k\sim\mathcal{N}(0,Q)
$$

## 5. 状態ヤコビアン

$$
F_k = \frac{\partial f}{\partial \mathbf{x}} =
\begin{bmatrix}
1 & \Delta t & 0 & 0 \\
-\Delta t\,\omega_k^2 & 1 & 0 & -2\Delta t\,\omega_k d_k \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

## 6. 観測方程式
観測信号 $y_k$ は

$$
y_k = h(\mathbf{x}_k) + v_k
$$
$$
h(\mathbf{x}_k) = d_k + c_k
$$
$$
v_k \sim \mathcal{N}(0,R)
$$

観測ヤコビアン:

$$
H_k = \frac{\partial h}{\partial \mathbf{x}} = \begin{bmatrix}1 & 0 & 1 & 0\end{bmatrix}
$$

## 7. EKF更新式
予測:

$$
\hat{\mathbf{x}}_{k|k-1}=f(\hat{\mathbf{x}}_{k-1|k-1})
$$
$$
P_{k|k-1}=F_k P_{k-1|k-1}F_k^T + Q
$$

イノベーション:

$$
\tilde y_k = y_k - h(\hat{\mathbf{x}}_{k|k-1})
$$
$$
S_k = H_k P_{k|k-1}H_k^T + R
$$

更新:

$$
K_k = P_{k|k-1}H_k^T S_k^{-1}
$$
$$
\hat{\mathbf{x}}_{k|k}=\hat{\mathbf{x}}_{k|k-1}+K_k\tilde y_k
$$
$$
P_{k|k}=(I-K_kH_k)P_{k|k-1}
$$

## 8. 初期値の推奨
- $\hat d_0=0,\hat{\dot d}_0=0,\hat c_0=0$
- $\hat f_0=0.6$ Hz（既存RLS条件と整合）
- $\hat\omega_0=2\pi\hat f_0$
- $P_0$ は対角大きめ（周波数は特に不確かさを大きめ）

## 9. チューニングの要点
- $q_\omega$:
  - 小さい: 安定だが追従が遅い
  - 大きい: 追従が速いが定常で揺れやすい
- $R$:
  - 小さすぎる: 観測ノイズを過信して発散リスク
  - 大きすぎる: 収束が遅い

## 10. 注意点
- 振幅が極小の区間では $\omega$ の可観測性が低下
- 高周波干渉時は $R$ と $q_{\dot d}, q_\omega$ の再調整が必要
- 実装では $\omega$ に上下限を設けると数値安定性が向上
