function [fitresult, gof] = motorFit(t, leftM)
%CREATEFIT(T,LEFTM)
%  Create a fit.
%
%  Data for 'untitled fit 1' fit:
%      X Input: t
%      Y Output: leftM
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 01-Mar-2024 14:11:15


%% Fit: 'untitled fit 1'.
[xData, yData] = prepareCurveData( t, leftM );

% Set up fittype and options.
ft = fittype( 'exp2' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [1.38459033484506 -0.000226093526123982 -0.986964610236507 -0.00215883214747229];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, xData, yData );
legend( h, 'leftM vs. t', 'untitled fit 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 't', 'Interpreter', 'none' );
ylabel( 'leftM', 'Interpreter', 'none' );
grid on

