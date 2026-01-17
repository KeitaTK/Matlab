"""
Ardupilot X-Frame Quadcopter Roll Dynamics Model (Python)

Motor configuration:
- Motor 1: Front Right (CCW)  - 右前
- Motor 2: Rear Left (CCW)    - 左後
- Motor 3: Front Left (CW)    - 左前  
- Motor 4: Rear Right (CW)    - 右後

Original MATLAB code converted to Python using SymPy
"""

import sympy as sp
import numpy as np

def create_roll_dynamics():
    """SymPyを使用してロール軸動力学モデルを作成"""
    # シンボリック変数の定義
    phi, dphi = sp.symbols('phi dphi', real=True)
    F1, F2, F3, F4 = sp.symbols('F1 F2 F3 F4', real=True)
    J, Lroll = sp.symbols('J Lroll', real=True)

    # 運動エネルギー（ロール軸のみ）
    W = sp.Rational(1, 2) * J * dphi**2

    # ラグランジアン（ポテンシャルエネルギーは0）
    L_lagrange = W

    # ロールモーメント
    tau_roll = (F1 + F4 - F2 - F3) * Lroll

    # オイラー–ラグランジュ方程式
    dL_ddphi = sp.diff(L_lagrange, dphi)
    M = sp.diff(dL_ddphi, dphi)  # 慣性マトリックス（スカラー）
    ddphi_expr = tau_roll / M

    # 状態方程式 [dphi, ddphi]
    state_derivatives = sp.Matrix([dphi, ddphi_expr])

    # Python関数として生成（NumPy最適化）
    all_vars = [phi, dphi, F1, F2, F3, F4, J, Lroll]
    dynamics_func = sp.lambdify(all_vars, state_derivatives, modules='numpy')

    return dynamics_func, tau_roll

class ArdupilotRollDynamics:
    """
    Ardupilot X-Frame クアッドコプターのロール軸動力学モデル

    Motor configuration:
    - Motor 1: Front Right (CCW)
    - Motor 2: Rear Left (CCW)
    - Motor 3: Front Left (CW)
    - Motor 4: Rear Right (CW)
    """

    def __init__(self, J=0.01, Lroll=0.106):
        """
        パラメータ:
        - J: ロール軸慣性モーメント [kg·m²]
        - Lroll: ロールモーメントアーム長 [m]
        """
        self.J = J
        self.Lroll = Lroll
        self._dynamics_func, self._tau_roll_expr = create_roll_dynamics()

    def dynamics(self, state, forces):
        """
        状態方程式の計算

        Parameters:
        - state: [phi, dphi] - ロール角とロール角速度
        - forces: [F1, F2, F3, F4] - 各モーターの推力 [N]

        Returns:
        - state_dot: [dphi, ddphi] - 状態の微分
        """
        phi, dphi = state
        F1, F2, F3, F4 = forces
        result = self._dynamics_func(phi, dphi, F1, F2, F3, F4, self.J, self.Lroll)
        # NumPy配列の要素を .item() でスカラー取得して返す
        return np.array([result[0].item(), result[1].item()])

    def roll_moment(self, forces):
        """
        ロールモーメントの計算

        Parameters:
        - forces: [F1, F2, F3, F4] - 各モーターの推力 [N]

        Returns:
        - tau_roll: ロールモーメント [N·m]
        """
        F1, F2, F3, F4 = forces
        return (F1 + F4 - F2 - F3) * self.Lroll

    def simulate_step(self, state, forces, dt):
        """
        4次ルンゲクッタ法（RK4）による1ステップシミュレーション

        Parameters:
        - state: [phi, dphi] - 現在の状態
        - forces: [F1, F2, F3, F4] - モーター推力
        - dt: 時間ステップ [s]

        Returns:
        - next_state: [phi_next, dphi_next] - 次の状態
        """
        # f(x, u) = 状態方程式
        def f(s):
            return self.dynamics(s, forces)

        k1 = f(state)
        k2 = f(state + 0.5 * dt * k1)
        k3 = f(state + 0.5 * dt * k2)
        k4 = f(state + dt * k3)
        next_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return next_state

    def set_parameters(self, J=None, Lroll=None):
        """パラメータの更新"""
        if J is not None:
            self.J = J
        if Lroll is not None:
            self.Lroll = Lroll

def test_roll_dynamics():
    """動力学モデルのテスト"""
    print("=== Ardupilot X-Frame Roll Dynamics Test ===\n")

    quad = ArdupilotRollDynamics(J=0.01, Lroll=0.106)

    phi_test = 0.0      # ロール角 [rad]
    dphi_test = 0.0     # ロール角速度 [rad/s]
    F_hover = 2.45      # ホバー推力 [N]

    forces = [F_hover + 0.5, F_hover - 0.5, F_hover - 0.5, F_hover + 0.5]
    state = [phi_test, dphi_test]

    state_dot = quad.dynamics(state, forces)
    tau_roll = quad.roll_moment(forces)

    print(f"初期状態: φ={phi_test:.3f} rad, dφ/dt={dphi_test:.3f} rad/s")
    print(f"推力: F1={forces[0]:.2f}, F2={forces[1]:.2f}, F3={forces[2]:.2f}, F4={forces[3]:.2f} [N]")
    print(f"ロールモーメント: {tau_roll:.4f} [N·m]")
    print(f"ロール角加速度: {state_dot[1]:.2f} [rad/s²]")

    return quad

if __name__ == "__main__":
    test_roll_dynamics()
