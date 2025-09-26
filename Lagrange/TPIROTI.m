syms m_p m_w
syms I_p I_w
syms r_p r_w
syms theta_p(t) theta_w(t)
syms g

omegap = diff(theta_p,t);
omegaw = diff(theta_w,t);

x_w = r_w*(theta_p + theta_w);
y_w = r_w;

x_p = x_w + r_p*sin(theta_p);
y_p = y_w + r_p*cos(theta_p);

v_xw = diff(x_w,t);
v_yw = diff(y_w,t);

v_xp = diff(x_p,t);
v_yp = diff(y_p,t);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Tw1 = m_w*v_xw^2/2 + m_w*v_yw^2/2;
Tw2 = I_w*(omegaw+omegap)^2/2;

Tp1 = m_p*v_xp^2/2 + m_w*v_yp^2/2;
Tp2 = I_p*omegap^2/2;

Up1 = m_p*g*y_p;

L = (Tw1 + Tw2 + Tp1 + Tp2) - (Up1);

eqn = [functionalDerivative(L, theta_p) == 0 ;
         functionalDerivative(L, theta_w) == 0];
eqn = simplify(eqn);