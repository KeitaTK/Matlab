% 重要 - このスクリプトは MATLAB Symbolic Math Toolbox を必要とし、実行に約3時間かかります

% 局所NED（北・東・下）地球接平面座標とXYZ機体固定座標を用いた航法EKFの導出
% 速度および位置観測の逐次融合
% 真対気速度（TAS）の融合
% 磁束（磁力計）観測の逐次融合
% オプティカルフローから得る視線（LOS）角速度の逐次融合
% センサは機体Z軸に整列していると仮定し、わずかなミスアライメントを許容
% 22状態の構成
% IMUデータは時間刻み dt で一定周期に到着すると仮定
% IMUのΔ角・Δ速度は観測ではなく時変パラメータとして扱う

% Author:  Paul Riseborough
% 著者: Paul Riseborough

% 状態ベクトル:
% クォータニオン (q0, q1, q2, q3)
% 速度 - m/s（北, 東, 下）
% 位置 - m（北, 東, 下）
% Δ角バイアス - rad（X, Y, Z）
% Δ速度バイアス - m/s（Z）
% 風ベクトル - m/s（北, 東）
% 地球磁場ベクトル - ミリガウス（北, 東, 下）
% 機体磁場ベクトル - ミリガウス（X, Y, Z）

% 観測量:
% NED速度 - m/s
% NED位置 - m
% 真対気速度 - m/s
% XYZ磁束（磁力計） - ミリガウス
% 下向きオプティカルフローセンサによるXY視線角速度
% 地表までの距離（レンジ）観測

% 時変パラメータ:
% 機体系XYZのΔ角測定 - rad
% 機体系XYZのΔ速度測定 - m/s

clear all;

%% 記号変数と定数の定義
syms dax day daz real % IMUの機体系Δ角測定 - rad
syms dvx dvy dvz real % IMUの機体系Δ速度測定 - m/s
syms q0 q1 q2 q3 real % 機体系の姿勢（局所NEDに対する）を定めるクォータニオン
syms vn ve vd real % NED速度 - m/s
syms pn pe pd real % NED位置 - m
syms dax_b day_b daz_b real % Δ角バイアス - rad
syms dvz_b real % Δ速度バイアス - m/s
syms dt real % IMUの時間刻み - s
syms gn ge gd real % NED重力ベクトル - m/s^2
syms daxCov dayCov dazCov dvxCov dvyCov dvzCov real; % IMUのΔ角・Δ速度測定の分散
syms vwn vwe real; % 北東（NE）風速 - m/s
syms magX magY magZ real; % 機体系XYZの磁場測定 - ミリガウス
syms magN magE magD real; % 地球固定NED磁場成分 - ミリガウス
syms R_VN R_VE R_VD real % NED速度測定の分散 - (m/s)^2
syms R_PN R_PE R_PD real % NED位置測定の分散 - m^2
syms R_TAS real  % 真対気速度測定の分散 - (m/s)^2
syms R_MAG real  % 磁束測定の分散 - ミリガウス^2
syms R_LOS real % 視線角速度測定の分散 (rad/s)^2
syms R_RNG real % レーザレンジファインダ観測の分散
syms ptd real % 地形のD軸位置
syms BCXinv BCYinv real % 機体系X/Y軸方向の風相対運動に対する弾道係数の逆数
syms rho real % 空気密度 (kg/m^3)
syms R_ACC real % 加速度計測定の分散 (m/s^2)^2
syms Kacc real % マルチローターにおける水平加速度と最大速度の比
syms decl real; % 地球磁場の磁偏角（真北から）
syms R_MAGS real; % 磁偏差（方位）測定の分散

%% プロセス方程式の定義

% 状態ベクトルと状態数の定義
stateVector = [q0;q1;q2;q3;vn;ve;vd;pn;pe;pd;dax_b;day_b;daz_b;dvz_b;vwn;vwe;magN;magE;magD;magX;magY;magZ];
nStates=numel(stateVector);

% 測定されたΔ角・Δ速度ベクトルの定義
da = [dax; day; daz];
dv = [dvx; dvy; dvz];

% Δ角・Δ速度バイアス誤差の定義
da_b = [dax_b; day_b; daz_b];
dv_b = [0; 0; dvz_b];

% 機体系→航法系の方向余弦行列の導出
Tbn = Quat2Tbn([q0,q1,q2,q3]);

% バイアス補正済みΔ角・Δ速度の定義
dAngCor = da - da_b;
dVelCor = dv - dv_b;

% クォータニオン回転ベクトルの定義
quat = [q0;q1;q2;q3];

% 姿勢更新方程式の定義
delQuat = [1;
    0.5*dAngCor(1);
    0.5*dAngCor(2);
    0.5*dAngCor(3);
    ];
qNew = QuatMult(quat,delQuat);

% 速度更新方程式の定義
vNew = [vn;ve;vd] + [gn;ge;gd]*dt + Tbn*dVelCor;

% 位置更新方程式の定義
pNew = [pn;pe;pd] + [vn;ve;vd]*dt;

% IMUバイアス誤差の更新方程式の定義
dabNew = [dax_b; day_b; daz_b];
dvbNew = dvz_b;

% 風速の更新方程式の定義
vwnNew = vwn;
vweNew = vwe;

% 地球磁場の更新方程式の定義
magNnew = magN;
magEnew = magE;
magDnew = magD;

% 機体磁場の更新方程式の定義
magXnew = magX;
magYnew = magY;
magZnew = magZ;

% プロセス方程式の出力ベクトルを定義
processEqns = [qNew;vNew;pNew;dabNew;dvbNew;vwnNew;vweNew;magNnew;magEnew;magDnew;magXnew;magYnew;magZnew];

%% 状態遷移行列の導出

% 状態遷移行列の導出
F = jacobian(processEqns, stateVector);
[F,SF]=OptimiseAlgebra(F,'SF');

%% 予測共分散の導出
% コード中で標準の行列演算を用いる場合に比べ、浮動小数点演算回数を6倍以上削減

% 制御（擾乱）ベクトルを定義。バイアスを除去した後のΔ角・Δ速度の「ノイズ」により、慣性解の誤差が増大すると仮定
% センサバイアスは状態方程式で扱っているためこの仮定で問題なし
distVector = [da;dv];

% 制御（擾乱）影響行列の導出
G = jacobian(processEqns, distVector);
[G,SG]=OptimiseAlgebra(G,'SG');

% 状態誤差行列の導出
imuNoise = diag([daxCov dayCov dazCov dvxCov dvyCov dvzCov]);
Q = G*imuNoise*transpose(G);
[Q,SQ]=OptimiseAlgebra(Q,'SQ');

% 文字列を用いて記号的な共分散行列を定義
% '_l_' は '( '、'_c_' は ','、'_r_' は ')' を表す
% 後で置換して実行可能なコードを生成
for rowIndex = 1:nStates
    for colIndex = 1:nStates
        eval(['syms OP_l_',num2str(rowIndex),'_c_',num2str(colIndex), '_r_ real']);
        eval(['P(',num2str(rowIndex),',',num2str(colIndex), ') = OP_l_',num2str(rowIndex),'_c_',num2str(colIndex),'_r_;']);
    end
end

% 標準式により予測共分散行列を導出
PP = F*P*transpose(F) + Q;

% 共通式を抽出して計算を最適化
[PP,SPP]=OptimiseAlgebra(PP,'SPP');

%% 速度・位置観測の逐次融合の導出
H_VN= jacobian(vn,stateVector); % 観測ヤコビアン
K_VN = (P*transpose(H_VN))/(H_VN*P*transpose(H_VN) + R_VN);

H_VE= jacobian(ve,stateVector); % 観測ヤコビアン
K_VE = (P*transpose(H_VE))/(H_VE*P*transpose(H_VE) + R_VE);

H_VD= jacobian(vd,stateVector); % 観測ヤコビアン
K_VD = (P*transpose(H_VD))/(H_VD*P*transpose(H_VD) + R_VD);

H_PN= jacobian(pn,stateVector); % 観測ヤコビアン
K_PN = (P*transpose(H_PN))/(H_PN*P*transpose(H_PN) + R_PN);

H_PE= jacobian(pe,stateVector); % 観測ヤコビアン
K_PE = (P*transpose(H_PE))/(H_PE*P*transpose(H_PE) + R_PE);

H_PD= jacobian(pd,stateVector); % 観測ヤコビアン
K_PD = (P*transpose(H_PD))/(H_PD*P*transpose(H_PD) + R_PD);

% H と K を1つにまとめる（ただし単一ステップ融合には使用不可）
% 各行/列を個別の融合ステップで用いる必要あり
H_VP  = [H_VN;H_VE;H_VD;H_PN;H_PE;H_PD];
clear    H_VN H_VE H_VD H_PN H_PE H_PD
K_VP = [K_VN,K_VE,K_VD,K_PN,K_PE,K_PD];
clear   K_VN K_VE K_VD K_PN K_PE K_PD
[K_VP,SK_VP]=OptimiseAlgebra(K_VP,'SK_VP');

%% 真対気速度観測の融合の導出
VtasPred = sqrt((vn-vwn)^2 + (ve-vwe)^2 + vd^2); % 予測観測
H_TAS = jacobian(VtasPred,stateVector); % 観測ヤコビアン
[H_TAS,SH_TAS]=OptimiseAlgebra(H_TAS,'SH_TAS'); % 計算の最適化
K_TAS = (P*transpose(H_TAS))/(H_TAS*P*transpose(H_TAS) + R_TAS);[K_TAS,SK_TAS]=OptimiseAlgebra(K_TAS,'SK_TAS'); % カルマンゲインベクトル

%% 磁場観測の融合の導出
magMeas = transpose(Tbn)*[magN;magE;magD] + [magX;magY;magZ]; % 予測観測
H_MAG = jacobian(magMeas,stateVector); % 観測ヤコビアン
[H_MAG,SH_MAG]=OptimiseAlgebra(H_MAG,'SH_MAG');

K_MX = (P*transpose(H_MAG(1,:)))/(H_MAG(1,:)*P*transpose(H_MAG(1,:)) + R_MAG); % カルマンゲインベクトル
[K_MX,SK_MX]=OptimiseAlgebra(K_MX,'SK_MX');
K_MY = (P*transpose(H_MAG(2,:)))/(H_MAG(2,:)*P*transpose(H_MAG(2,:)) + R_MAG); % カルマンゲインベクトル
[K_MY,SK_MY]=OptimiseAlgebra(K_MY,'SK_MY');
K_MZ = (P*transpose(H_MAG(3,:)))/(H_MAG(3,:)*P*transpose(H_MAG(3,:)) + R_MAG); % カルマンゲインベクトル
[K_MZ,SK_MZ]=OptimiseAlgebra(K_MZ,'SK_MZ');

%% オプティカルフロー観測の逐次融合の導出

% 平坦地表かつカメラ軸が機体系に整列していると仮定し、機体から視野中心までの距離を計算
range = ((ptd - pd)/Tbn(3,3));
% 機体系での相対速度を計算
relVelBody = transpose(Tbn)*[vn;ve;vd];
% 距離で割り、X/Y軸に対する視線角速度（予測）を算出
% これは機体角速度の影響を補償したオプティカルフローレート
losRateX = +relVelBody(2)/range;
losRateY = -relVelBody(1)/range;

H_LOS = jacobian([losRateX;losRateY],stateVector); % 観測ヤコビアン
[H_LOS,SH_LOS]=OptimiseAlgebra(H_LOS,'SH_LOS');

% 共通式抽出のため K 行列を結合（ただし単一ステップ融合には使用不可）
K_LOSX = (P*transpose(H_LOS(1,:)))/(H_LOS(1,:)*P*transpose(H_LOS(1,:)) + R_LOS); % カルマンゲインベクトル
K_LOSY = (P*transpose(H_LOS(2,:)))/(H_LOS(2,:)*P*transpose(H_LOS(2,:)) + R_LOS); % カルマンゲインベクトル
K_LOS = [K_LOSX,K_LOSY];
simplify(K_LOS);
[K_LOS,SK_LOS]=OptimiseAlgebra(K_LOS,'SK_LOS');

%% レーザレンジファインダ観測の融合の導出

% 平坦地表かつセンサが機体Z軸に整列と仮定し、距離を計算
range = ((ptd - pd)/Tbn(3,3));
H_RNG = jacobian(range,stateVector); % 観測ヤコビアン
[H_RNG,SH_RNG]=OptimiseAlgebra(H_RNG,'SH_RNG');

% カルマンゲインの計算と式の最適化
K_RNG = (P*transpose(H_RNG))/(H_RNG*P*transpose(H_RNG) + R_RNG);
[K_RNG,SK_RNG]=OptimiseAlgebra(K_RNG,'SK_RNG');

%% 横方向機体加速度の融合（マルチローターのみ）

% 機体系X/Y軸の対気速度と抗力の関係から、主推力が機体Z軸方向のマルチローターにおける横加速度を予測

vrel = transpose(Tbn)*[(vn-vwn);(ve-vwe);vd]; % 予測される風相対速度

% 正方向への飛行を仮定して抗力を計算
% 記号導出に符号関数を入れてディラック関数を生まないよう、実装側で符号を扱う
% accXpred = -0.5*rho*vrel(1)*vrel(1)*BCXinv; % X軸方向の予測加速度（機体座標）
% accYpred = -0.5*rho*vrel(2)*vrel(2)*BCYinv; % Y軸方向の予測加速度（機体座標）

% 線形推定器のため簡易な粘性抗力モデルを使用
% 速度から加速度への導関数を速度範囲で平均して用いる
% 予測観測の計算は実装側で非線形式を用いる
accXpred = -Kacc*vrel(1); % X軸方向の予測加速度（機体座標）
accYpred = -Kacc*vrel(2); % Y軸方向の予測加速度（機体座標）

% X加速度融合の観測ヤコビアンおよびカルマンゲインを導出
H_ACCX = jacobian(accXpred,stateVector); % 観測ヤコビアン
[H_ACCX,SH_ACCX]=OptimiseAlgebra(H_ACCX,'SH_ACCX'); % 計算の最適化
K_ACCX = (P*transpose(H_ACCX))/(H_ACCX*P*transpose(H_ACCX) + R_ACC);
[K_ACCX,SK_ACCX]=OptimiseAlgebra(K_ACCX,'SK_ACCX'); % カルマンゲインベクトル

% Y加速度融合の観測ヤコビアンおよびカルマンゲインを導出
H_ACCY = jacobian(accYpred,stateVector); % 観測ヤコビアン
[H_ACCY,SH_ACCY]=OptimiseAlgebra(H_ACCY,'SH_ACCY'); % 計算の最適化
K_ACCY = (P*transpose(H_ACCY))/(H_ACCY*P*transpose(H_ACCY) + R_ACC);
[K_ACCY,SK_ACCY]=OptimiseAlgebra(K_ACCY,'SK_ACCY'); % カルマンゲインベクトル

%% 磁偏差（方位）観測の融合の導出

% 地球座標系での測定磁場
magMeas = transpose(Tbn)*[magN;magE;magD] + [magX;magY;magZ]; % 予測観測
% 機体系での測定磁場
magMeasNED = Tbn*magMeas;
% 予測観測は、測定磁場の水平成分の磁北に対する角度
angMeas = tan(magMeasNED(2)/magMeasNED(1)) - decl;
H_MAGS = jacobian(angMeas,stateVector); % 観測ヤコビアン
H_MAGS = simplify(H_MAGS);
[H_MAGS,SH_MAGS]=OptimiseAlgebra(H_MAGS,'SH_MAGS');
K_MAGS = (P*transpose(H_MAGS))/(H_MAGS*P*transpose(H_MAGS) + R_MAGS);
[K_MAGS,SK_MAGS]=OptimiseAlgebra(K_MAGS,'SK_MAGS'); % カルマンゲインベクトル

%% 出力の保存と m/c コード断片への変換
nStates = 22;
fileName = strcat('SymbolicOutput',int2str(nStates),'.mat');
save(fileName);
SaveScriptCode(nStates);
ConvertToM(nStates);
ConvertToC(nStates);