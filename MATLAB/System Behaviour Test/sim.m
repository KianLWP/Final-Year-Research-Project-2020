function dy = sim(y,u)
%declare constants, taken from other script
g = 9.81;
r = 0.04; 
l = 0.0530; 
m_b = 0.9692; 
m_w = 0.0200; 
I_b = 0.0085; 
I_w =  1.0271e-05; 

K_m = 0.0067; 
R = 7.5; 
b_w = 0.001; %estimated

%state outputs
v_s = u;
dx = y(2);
theta = y(3);
dtheta = y(4);

%Equations of motion taken from derivations script
dy(1,1) = y(2);
dy(2,1) = ((2*((K_m^2*(dtheta - dx/r))/R + (K_m*v_s)/R))/r - (2*b_w*dx)/r^2 + dtheta^2*l*m_b*sin(theta) + (l*m_b*cos(theta)*((2*K_m^2*(dtheta - dx/r))/R + (2*K_m*v_s)/R - g*l*m_b*sin(theta)))/(m_b*l^2 + I_b))/(m_b + 2*m_w + (2*I_w)/r^2 - (l^2*m_b^2*cos(theta)^(2))/(m_b*l^2 + I_b));
dy(3,1) = y(4);
dy(4,1) =  -((2*K_m^2*(dtheta - dx/r))/R + (2*K_m*v_s)/R - g*l*m_b*sin(theta) + (l*m_b*cos(theta)*((2*((K_m^2*(dtheta - dx/r))/R + (K_m*v_s)/R))/r - (2*b_w*dx)/r^2 + dtheta^2*l*m_b*sin(theta)))/(m_b + 2*m_w + (2*I_w)/r^2))/(I_b + l^2*m_b - (l^2*m_b^2*cos(theta)^(2))/(m_b + 2*m_w + (2*I_w)/r^2));
