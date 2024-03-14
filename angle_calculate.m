clear
clf
load("data/angular_position.mat")
[peaks, peak_index] = findpeaks(angular_data);
t = linspace(0,0.05*98,98);

% Calculate periods between successive peaks
for i = 1:length(peak_index)-1
    period(i) = t(peak_index(i+1))-t(peak_index(i));
end
average_period = mean(period);

figure()
% Plot angular position data
plot(t, angular_data,'LineWidth', 1.5);
hold on
plot(t(peak_index), peaks, 'ro', 'MarkerFaceColor', 'r');
hold off

% Enhancements
xlabel('Time (s)');
ylabel('Angular Position (radians)');
title("Rocky's Angular Position While Swinging");
legend({'Measured Data', 'Peaks'}, 'Location', 'northeast');
ylim([-2,2.5])
grid on
