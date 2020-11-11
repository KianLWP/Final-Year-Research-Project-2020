% syms g K_m R r l m_b m_w I_b I_w 'real'
%declare constants
g = 9.81;
K_m = 0.007;
R = 5;
r = 0.04;
l = 0.04;
m_b = 0.9692;
m_w = 0.008;
I_b = 0.0085;
I_w = 1.0271e-05;


%% State Space Representation
A_22 =  -((2*K_m^2)/(R*r^2) + (2*K_m^2*l*m_b)/(R*r*(m_b*l^2 + I_b)))/...
        (m_b + 2*m_w + (2*I_w)/r^2 - (l^2*m_b^2)/(m_b*l^2 + I_b));
    
A_23 =  - (g*l^2*m_b^2)/...
        ((m_b*l^2 + I_b)*(m_b + 2*m_w + (2*I_w)/r^2 - (l^2*m_b^2)/(m_b*l^2 + I_b)));
    
A_24 = ((2*K_m^2)/(R*r) + (2*K_m^2*l*m_b)/(R*(m_b*l^2 + I_b)))/...
        (m_b + 2*m_w + (2*I_w)/r^2 - (l^2*m_b^2)/(m_b*l^2 + I_b));


A_42 = ((2*K_m^2)/(R*r) + (2*K_m^2*l*m_b)/(R*r^2*(m_b + 2*m_w + (2*I_w)/r^2)))/...
        (I_b + l^2*m_b - (l^2*m_b^2)/(m_b + 2*m_w + (2*I_w)/r^2));
    
A_43 = (g*l*m_b)/...
        (I_b + l^2*m_b - (l^2*m_b^2)/(m_b + 2*m_w + (2*I_w)/r^2));
    
A_44 =  - ((2*K_m^2)/R + (2*K_m^2*l*m_b)/(R*r*(m_b + 2*m_w + (2*I_w)/r^2)))/...
        (I_b + l^2*m_b - (l^2*m_b^2)/(m_b + 2*m_w + (2*I_w)/r^2));

A_ = [0   ,   1   ,   0   ,   0   ;
     0   ,   A_22,   A_23,   A_24;
     0   ,   0   ,   0   ,   1   ;
     0   ,   A_42,   A_43,   A_44];

 
B_2 = ((2*K_m)/(R*r) + (2*K_m*l*m_b)/(R*(m_b*l^2 + I_b)))/...
    (m_b + 2*m_w + (2*I_w)/r^2 - (l^2*m_b^2)/(m_b*l^2 + I_b));

B_4 =  - ((2*K_m)/R + (2*K_m*l*m_b)/(R*r*(m_b + 2*m_w + (2*I_w)/r^2)))/...
        (I_b + l^2*m_b - (l^2*m_b^2)/(m_b + 2*m_w + (2*I_w)/r^2));

    
B_ = [0  ;
     B_2;
     0  ;
     B_4];
 
% Flow board output: 
% sensor's x pos and height -> robot's x + angle
% C = [1, 0, 0, 0;
%     0, 0, 1, 0];
% MPU6050 output: 
%x, y, z acceleration from acceleromter -> angle + velocity
%angular velocity from gyroscope
% C = [0, 0, 0, 0;
%      0, 0, 0, 0;
%      0, 0, 1, 0;
%      0, 0, 0, 0];

%Remove position
A = A_(2:end,2:end);
B = B_(2:end);
%Angle control
D = zeros(3,1);

C = [1 0 0 
     0 1 0
     0 0 1];

D_ = 0;
C_1 = [1 0 0];
C_2 = [0 1 0];
C_3 = [0 0 1];
% [num,den]=ss2tf(A,B,C,D);
% [z, p, k]=ss2zp(A,B,C,D);
% C = eye(4);
% D = zeros(4,1);
%% Build System
sys = ss(A,B,C,D);

%stability
% a = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
% impulse(sys);
% saveas(a, 'OpenLoop.png');

e = eig(A); % one pole on right hand side of S-plane; therefore not stable
poles = pole(sys);

% rank of both is 3; therefore controlable and observable
co = ctrb(sys);
obs = obsv(sys);

sys_1 = ss(A,B,C_1, D_);
sys_2 = ss(A,B,C_2, D_);
sys_3 = ss(A,B,C_3, D_);
obs_1 = obsv(sys_1);
obs_2 = obsv(sys_2);
obs_3 = obsv(sys_3);


% initial conditions
ang = deg2rad(5);
x0 = [0 -ang 0];
dist = [0 0 0];
%% Controller
%State-Space 
des_pole = [-1; -1; -1]*2;
K_ss = acker(A, B, des_pole)

%LQR
Q = eye(3);
Q(1,1) = 1;
Q(3,3) = 100;
Q(2,2) = 10;
R = 1;
K_lqr = lqr(A,B,Q,R)

%% Feedback
t = 0:0.01:20;
u = zeros(size(t));
sys_cl = ss(A-B*K_ss,B,C,0);
% lsim(sys_cl,u,t,x0);
