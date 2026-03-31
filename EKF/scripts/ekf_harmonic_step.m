function [x_upd, P_upd, debug] = ekf_harmonic_step(x_prev, P_prev, y_meas, dt, Q, R, opts)
% ekf_harmonic_step
% One EKF step for harmonic-oscillator disturbance model.
% State: x = [d; d_dot; c; omega]
% Measurement: y = d + c + v

if nargin < 7
    opts = struct();
end
if ~isfield(opts, 'omega_min'), opts.omega_min = 0.2; end
if ~isfield(opts, 'omega_max'), opts.omega_max = 15.0; end
if ~isfield(opts, 'symmetrizeP'), opts.symmetrizeP = true; end

% Prediction (Euler discretization)
d = x_prev(1);
d_dot = x_prev(2);
c = x_prev(3);
omega = x_prev(4);

x_pred = zeros(4,1);
x_pred(1) = d + dt * d_dot;
x_pred(2) = d_dot + dt * (-(omega^2) * d);
x_pred(3) = c;
x_pred(4) = omega;

% Keep frequency in a valid range
x_pred(4) = min(max(x_pred(4), opts.omega_min), opts.omega_max);

% State Jacobian F = df/dx
F = [
    1,              dt, 0,                 0;
    -dt * omega^2,   1, 0, -2 * dt * omega * d;
    0,               0, 1,                 0;
    0,               0, 0,                 1
];

P_pred = F * P_prev * F' + Q;

% Measurement model: y = d + c
H = [1, 0, 1, 0];
y_pred = x_pred(1) + x_pred(3);
innov = y_meas - y_pred;

S = H * P_pred * H' + R;
K = (P_pred * H') / S;

x_upd = x_pred + K * innov;
P_upd = (eye(4) - K * H) * P_pred;

if opts.symmetrizeP
    P_upd = 0.5 * (P_upd + P_upd');
end

% Clamp omega after update as well
x_upd(4) = min(max(x_upd(4), opts.omega_min), opts.omega_max);

debug.x_pred = x_pred;
debug.P_pred = P_pred;
debug.innov = innov;
debug.S = S;
debug.K = K;
debug.y_pred = y_pred;
end
