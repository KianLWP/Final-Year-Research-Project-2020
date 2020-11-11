clear
clc
%% Controller Performance
% % file = 'SteadyState.xlsx';
% % file = 'InitialAngle.xlsx';
% % file = 'AddedWeight.xlsx';
% % file = 'Disturbance.xlsx';
% file = 'Wind.xlsx';
% 
% voltage = xlsread(file,'A:A');
% angle = xlsread(file,'B:B');
% time = xlsread(file,'D:D');
% 
% a = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
% plot(time, angle);
% ylabel('Angle (rads)');
% xlabel('Time (s)');
% xlim([0 time(end)]); %time(end)
% saveas(a, 'WindAng.png');
% 
% b = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
% plot(time, voltage);
% ylabel('Output Voltage (V)');
% xlabel('Time (s)');
% xlim([0 time(end)]);
% saveas(b, 'WeightVolt.png');

%% Flow Angle
file2 = 'FlowAngleTracking.xlsx';

angle = xlsread(file2,'A:A');
flowangle = xlsread(file2,'B:B');
time2 = xlsread(file2,'D:D');

c = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
yyaxis left
plot(time2, angle);
xlabel('Time (s)');
ylabel('IMU Angle (rad)');

yyaxis right
plot(time2, flowangle);
ylabel('Flow Angle (rad)');
saveas(c, 'FlowAngle.png');