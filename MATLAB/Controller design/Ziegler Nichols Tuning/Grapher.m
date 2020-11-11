file = 'ControllerDesign.xlsx';
pic = 'ControllerTest.png';

angle = xlsread(file,'B:B');

time = xlsread(file,'D:D');

a = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
plot(time, angle);
ylabel('Angle (rads)');
xlabel('Time (s)');
xlim([0 time(end)]);
ylim([-0.1 0.1]);
saveas(a, pic);
