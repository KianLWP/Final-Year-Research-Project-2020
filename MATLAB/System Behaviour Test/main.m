clear all
clc 
close all  

%% Simulation at the up position
tspan = 0:0.01:35;  %robotSimulation time
y0 = [0,0,pi/180,0]; %initial states
[t,y] = ode45(@(t,y)robotSim(y,0),tspan,y0); %compute robotSimulation


%Plot
a = figure('Renderer', 'painters', 'Position', [10 10 1100 450]); %set figure size
yyaxis left
plot(t, y(:,3));
ylabel('Robot Angle (rad)');
xlabel('Time (s)');
ylim([0 2*pi]);

yyaxis right
plot(t, y(:,1));
ylabel('Robot Position (m)');
ylim([-0.1 0.1]);
% saveas(a, 'PlotUp.png');

% % Animation, uncomment to see robotSimulation
% figure();
% for k=1:length(t)
%    animation(y(k,:));
% end

%% Simulation at the down position
y0 = [0,0,pi/180+pi,0];
[t,y] = ode45(@(t,y)robotSim(y,0),tspan,y0);

%Plot

b = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
yyaxis left
plot(t, y(:,3));
ylabel('Robot Angle (rad)');
xlabel('Time (s)');
% ylim([pi-0.15 pi+0.15]);
ylim([pi-0.05 pi+0.05]);

yyaxis right
plot(t, y(:,1));
ylabel('Robot Position (m)');
% ylim([-5*10^-3 5*10^-3]);
ylim([-2*10^-3 2*10^-3]);
% saveas(b, 'PlotDown.png');

% %Animation
% figure();
% for k=1:length(t)
%    animation(y(k,:));
% end


%% Simulation of 12V actuation
y0 = [0,0,pi,0];
[t,y] = ode45(@(t,y)robotSim(y,12),tspan,y0);


%Plot
c = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
yyaxis left
plot(t, y(:,3));
ylabel('Robot Angle (rad)');
xlabel('Time (s)');

yyaxis right
plot(t, y(:,1));
ylabel('Robot Position (m)');
% saveas(c, 'Plot12V.png');

% % Animation
% figure();
% for k=1:length(t)
%    animation(y(k,:));
% end


%% Simulation of 1V actuation
y0 = [0,0,pi,0];
[t,y] = ode45(@(t,y)robotSim(y,1),tspan,y0);


d = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
yyaxis left
plot(t, y(:,3));
ylabel('Robot Angle (rad)');
xlabel('Time (s)');
ylim([pi-0.007 pi+0.001]);

yyaxis right
plot(t, y(:,1));
ylabel('Robot Position (m)');
ylim([0 3]);
% saveas(d, 'Plot1V.png');

% % Animation
% figure();
% for k=1:length(t)
%    animation(y(k,:));
% end

