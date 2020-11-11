syms x y theta dx dy dtheta ddx ddy ddtheta 'real'
syms m_b m_w g l r I_b I_w b_w 'real' %r = radius of wheel, l = length, b = body, w = wheel
syms tau_L tau_R tau_w'real' %left and right wheel torque
syms K_m b_m I_m v_s R phi dphi ddphi 'real'

tau = [ddx*(m_b + 2*m_w + (2*I_w)/r^2) + (2*b_w*dx)/r^2 + ddtheta*l*m_b*cos(theta) - dtheta^2*l*m_b*sin(theta); ddtheta*(m_b*l^2 + I_b) + ddx*l*m_b*cos(theta) - g*l*m_b*sin(theta)];
%Small angle approximation
tau_1 = subs(tau(1), [dtheta^2, cos(theta), sin(theta),b_w], [0, 1, theta,0]);
tau_2 = subs(tau(2), [dtheta^2, cos(theta), sin(theta),b_w], [0, 1, theta,0]);
%Left and right wheel same torque
Q1 = subs(Q(1), [tau_L, tau_R], [tau_w, tau_w]);
Q2 = subs(Q(2), [tau_L, tau_R], [tau_w, tau_w]);
%Substitute and rearrange
eqn1 = (tau_1 == Q1);
eqnX = solve(eqn1, ddx);

eqn2 = (tau_2 == Q2);
eqnTh = solve(eqn2, ddtheta);

eqnA = subs(eqn1, [ddtheta], [eqnTh]);
eqnB = subs(eqn2, [ddx], [eqnX]);

%Equations of motion
ddq_x = solve(eqnA,ddx)
ddq_th = solve(eqnB,ddtheta)

