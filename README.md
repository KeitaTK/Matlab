# ドローン姿勢制御・最適化プロジェクト README

本プロジェクトは、ラグランジュの運動方程式に基づくクアッドコプター（ドローン）の姿勢制御・最適化を扱い、理論モデルと実験データの統合を目指します。

## フォルダ構成の概要

- Lagrange/
  - chain_rule.mlx — 連鎖律の計算を行うMATLABライブスクリプト
  - GURAFU.m — グラフ描画用スクリプト
  - HURIKO_SHITA.m — 振り子シミュレーションプログラム
  - matsuura_1218.m、sennsei_1218.m — 特定日付のシミュレーション実行スクリプト
  - quad_Lag.m、quad_Lag.mlx — 四輪機（クアッドコプター）のラグランジュ方程式モデル
  - Quad_simu1.m — クアッドコプターのシミュレーション実行スクリプト
  - TPIROTI.m、TPIROTI.mlx — 姿勢角（ロール/ピッチ/ヨー）計算プログラム
  - yomikomi.m — データ読み込み用スクリプト
  - Quad_0.mlx — クアッドコプターの基本モデル
  - Quad_Lag_2.mlx ～ Quad_Lag_6.mlx — ラグランジュ方程式モデルの各バージョン
- position/
  - drone_controller.mlx — ドローン制御アルゴリズム
  - drone_controller_original.mlx — オリジナルの制御アルゴリズム
- ラグランジュの運動方程式から姿勢シミュレーション/
  - 最適化/
    - Quad_lsqnonlin1.m — パラメータ最適化（J_xx, J_yy, J_zz, mu_1, mu_2）
    - アーカイブ/
      - Quad_lsqnonlin1.m — 以前のバージョン
      - Quad_lsqnonlin2.m — プロペラ位置も考慮
      - Quad_lsqnonlin3.m — オフセット位置(OF_x, OF_y)も含めた拡張
  - 慣性座標系/
    - Quad_lsqnonlin1.m — 慣性座標系でのパラメータ最適化

## 主要コードの詳細

- 最適化（Quad_lsqnonlinシリーズ）
  - 実飛行データに対し、慣性モーメント(J_xx, J_yy, J_zz)や摩擦係数(mu_1, mu_2)等を最小二乗でフィッティング
  - MATLABの lsqnonlin（レーベンバーグ・マルカート法）を使用
- ラグランジュ方程式モデル（quad_Lagシリーズ）
  - ラグランジュの運動方程式によるドローンの運動モデル化
  - バージョンごとに仮定や改良が異なる
- 姿勢計算（TPIROTIシリーズ）
  - ロール・ピッチ・ヨー角の計算や座標変換

## データファイル（.mat）

- Omega.mat, dX.mat, R_BtoI.mat — 角速度、状態変化、座標変換行列
- qqq1.mat, qqq2.mat, qqq3.mat — シミュレーション結果/中間データ
- HURIKO_SHITA.mat — 振り子シミュレーションのデータ

## 実行方法（例）

- モデル/シミュレーション
  - Lagrange/Quad_simu1.m または quad_Lag.mlx を開いて実行
- パラメータ最適化
  - ラグランジュの運動方程式から姿勢シミュレーション/最適化/Quad_lsqnonlin1.m を実行
  - アーカイブ/各バージョンや慣性座標系版も用途に応じて実行

## 必要環境

- MATLAB（R2020以降推奨）
- Optimization Toolbox（lsqnonlin使用）

## 回転行列・座標変換を扱う主なファイル

- Lagrange/TPIROTI.m, Lagrange/TPIROTI.mlx
  - ロール・ピッチ・ヨーから回転行列 R_BtoI, R_ItoB を計算。座標変換の実装を含む。
- Lagrange/quad_Lag.m, Lagrange/quad_Lag.mlx
  - 動力学モデル内で方向余弦行列（回転行列）を定義・使用。
- Lagrange/Quad_0.mlx, Lagrange/Quad_Lag_2.mlx ～ Quad_Lag_6.mlx
  - 各バージョンで回転行列・座標変換を用いた拡張モデル。
- ラグランジュの運動方程式から姿勢シミュレーション/慣性座標系/Quad_lsqnonlin1.m
  - 慣性座標系⇔機体座標系の変換を用いた最適化。
- Lagrange/yomikomi.m
  - R_BtoI などの座標変換関連データの読み込み（例: R_BtoI.mat）。
- データ: R_BtoI.mat
  - ボディ座標系→慣性座標系の回転行列サンプル。