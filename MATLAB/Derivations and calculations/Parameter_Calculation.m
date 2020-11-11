% Calculating inertia and location of COM
%% Size
l_rod = 0.165;
b_ = 0.13;
w_ = 0.065;
h_ = 3/1000;
r_motor = 37/1000; %assume uniform size (which it isnt)
d_motor = 7/1000; %shaft offset from COM
b_motor_mount = 40/1000;
h_motor_mount = 46/1000;
r_wheel = 40/1000;
r_mount_hub = 25.4/1000; 

%Platform distances
syms d0 d1 d2 d3 z_com 'real'
d0 = (7.577+31*sqrt(3)/2)/1000; %distance from axle to centre of bottom platform
d1 = 75/1000; %distance from bottom platform to middle
d2 = d1; %distance from middle to top

%% Mass
m_rod = 0.118*l_rod; %weight per m
m_panel = 1200*h_*b_*w_; %estimated high https://plasticsheetsshop.co.uk/the-weight-of-acrylic-sheet/
m_motor = 0.137; 
m_motor_mount = 156/1000;
m_wheel = 8/1000;
m_mount_hub = 12/1000;
%components
m_flow = 1.6/1000;
m_imu = 2/1000;
m_micro = 8/1000;

m_gimbal_shaft = 7800*(pi*0.004^2)*0.2;
m_gimbal_platform = 60/1000;
m_gimbal_support = 32/1000;
m_gimbal = m_gimbal_shaft + m_gimbal_platform + m_flow;
%total
m_robot = 4*m_rod + 3*m_panel + m_gimbal + 2*m_gimbal_support + m_imu + m_micro + 2*m_motor + 2*m_motor_mount;

%% Inertia and COM
%rods
I_rod = 1/12*m_rod*l_rod^2 + m_rod*(d0+0.075)^2;
I_rods = 4*I_rod;
z_rod = 4*m_rod*(d0+0.075);
%panels, assume panels are above motor, at halfway and end
I_panel = 1/12*m_panel*(h_^2+w_^2);
I_panels = 3*I_panel + m_panel*(d0^2 + (d0 + d1)^2 + (d0 + d1 + d2)^2); 
z_panel = m_panel*((d0 + (d0 + d1) + (d0 + d1 + d2)));

%gimbal calculations
I_gimbal = m_gimbal*(d0+d1+d2-21.5/1000)^2;
z_gimbal = m_gimbal*(d0+d1+d2-21.5/1000);

m_top = m_gimbal_support * (42*6)/(42*6+26.8*18);
m_bottom = m_gimbal_support-m_top;
I_top = 1/12*(m_top)*(0.042^2+0.006^2) + m_top*(d0+d1+d2-(1.5+3)/1000)^2;
I_bottom = 1/12*(m_bottom)*(0.0268^2+0.018^2) + m_bottom*(d0+d1+d2-(1.5+6+26.8/2)/1000)^2;
I_gimbal_support = 2*(I_top + I_bottom);
z_top = m_top*(d0+d1+d2-(1.5+3)/1000);
z_bottom = m_bottom*(d0+d1+d2-(1.5+6+26.8/2)/1000);
z_gimbal_support = 2*(z_top + z_bottom);
%electrical components
I_imu = 1/12*(m_imu)*(12^2+4^2)*10^-6 + m_imu*(d0+d1+5/1000)^2;
z_imu = m_imu*(d0+d1+5/1000);
I_micro = 1/12*(m_micro)*(18^2+4^2)*10^-6 + m_micro*(d0+d1+5/1000)^2;
z_micro = m_micro*(d0+d1+5/1000);

%motors
I_motor = 1/2*m_motor*r_motor^2 + m_motor*(7/1000)^2;
I_motors = 2*I_motor;
z_motor = -2*m_motor*(7/1000);

I_motor_mount = 2*1/12*m_motor_mount*(b_motor_mount^2+h_motor_mount^2);
z_motor_mount = 0;

%% Final Values
I_robot = I_rods + I_panels + I_gimbal + I_gimbal_support + I_imu + I_micro + I_motors ;
z_com = (z_rod + z_panel + z_gimbal + z_gimbal_support + z_imu + z_micro + z_motor + z_motor_mount)/m_robot;

I_wheel = 1/2*m_wheel*r_wheel^2;
I_mount_hub = 1/2*m_mount_hub*r_mount_hub^2;
I_w_ = I_wheel+I_mount_hub;
m_w_ = m_wheel+m_mount_hub;