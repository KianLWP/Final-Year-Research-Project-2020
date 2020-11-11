syms x y theta dx dy dtheta ddx ddy ddtheta 'real'
syms m_b m_w g l r I_b I_w b_w 'real' %r = radius of wheel, l = length, b = body, w = wheel
syms tau_L tau_R 'real' %left and right wheel torque
syms K_m b_m I_m v_s R phi dphi ddphi 'real'


tau = [ddx*(m_b + 2*m_w + (2*I_w)/r^2) + (2*b_w*dx)/r^2 + ddtheta*l*m_b*cos(theta) - dtheta^2*l*m_b*sin(theta); ddtheta*(m_b*l^2 + I_b) + ddx*l*m_b*cos(theta) - g*l*m_b*sin(theta)];
%Unlinearized EOM
Q1 = (((2*K_m^2*(dtheta - dx/r))/R + (2*K_m*v_s)/R)/r - (2*b_w*dx)/r^2 + dtheta^2*l*m_b*sin(theta) + (l*m_b*cos(theta)*((2*K_m^2*(dtheta - dx/r))/R + (2*K_m*v_s)/R - g*l*m_b*sin(theta)))/(m_b*l^2 + I_b))/(m_b + 2*m_w + (2*I_w)/r^2 - (l^2*m_b^2*cos(theta)^sym(2))/(m_b*l^2 + I_b));
Q2 = -((2*K_m^2*(dtheta - dx/r))/R + (2*K_m*v_s)/R - g*l*m_b*sin(theta) + (l*m_b*cos(theta)*(((2*K_m^2*(dtheta - dx/r))/R + (2*K_m*v_s)/R)/r - (2*b_w*dx)/r^2 + dtheta^2*l*m_b*sin(theta)))/(m_b + 2*m_w + (2*I_w)/r^2))/(I_b + l^2*m_b - (l^2*m_b^2*cos(theta)^sym(2))/(m_b + 2*m_w + (2*I_w)/r^2));
%Substitute and rearrange
eqn1 = (tau(1) == Q1);
eqnX = solve(eqn1, ddx);

eqn2 = (tau(2) == Q2);
eqnTh = solve(eqn2, ddtheta);

eqnA = subs(eqn1, [ddtheta], [eqnTh]);
eqnB = subs(eqn2, [ddx], [eqnX]);

%Equations of motion
ddq_x = solve(eqnA,ddx) 
ddq_th = solve(eqnB,ddtheta)