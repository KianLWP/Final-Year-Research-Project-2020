file = 'ZN_data.xlsx';

angle = xlsread(file,'A1:A536');
time = xlsread(file,'C1:C536');

[pks,locs] = findpeaks(angle,time);

tot = 0;
for c = 3:length(pks)
    dt = locs(c)-locs(c-1);
    tot = tot+dt;
end

Tu = tot/length(pks)

a = figure('Renderer', 'painters', 'Position', [10 10 1100 450]);
findpeaks(angle,time);
ylabel('Angle (rads)');
xlabel('Time (s)');

saveas(a, 'Ziegler-Nichols.png');