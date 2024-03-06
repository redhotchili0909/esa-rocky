load('data/motorCalibration.mat')
t = linspace(0,81*20, 81);
leftM = A(:, 1);
rightM = A(:, 2);

subplot(2,1,1)
plot(t, leftM);
title('Left Motor')
xlabel('Time (ms)')
ylabel('Speed (cm/s)')

subplot(2,1,2)
plot(t, rightM);
title('Right Motor')
xlabel('Time (ms)')
ylabel('Speed (cm/s)')


motorFit(t, leftM');
motorFit(t, rightM')



function [fitresult, gof] = motorFit(t, wheel)

[xData, yData] = prepareCurveData( t, wheel );

% Set up fittype and options.
ft = fittype( 'exp2' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [1.38459033484506 -0.000226093526123982 -0.986964610236507 -0.00215883214747229];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure();
h = plot( fitresult, xData, yData );
legend( h, 'Measured', 'Fit', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel('Time (ms)')
ylabel('Speed (cm/s)')
grid on
end
