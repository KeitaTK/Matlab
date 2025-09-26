syms theta(t) L m g
x = L*sin(theta);
y = L*cos(theta);
vx = diff(x,t);
vy = diff(y,t);

T = m*(vx^2+vy^2)/2;
T = simplify(T);

U = m*g*(L-y);
U = simplify(U);

L = T-U;

eqn = functionalDerivative(L,theta)==0;
isolate(eqn,diff(theta,t,t))