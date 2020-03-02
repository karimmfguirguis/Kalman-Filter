function [x, P]= KalmanPredict(x, P, F, Q, B, u)
% ----------------------------------------------------------------------
%                           Kalman Filter Prediction 
%
%   This function implements the kalman filter prediction. 
%  
%   Usage:  [x, P]= KalmanPredict(x, P, F, Q, B, u)
%           
%         x  - state vector
%         P  - covariance of the system state 
%         F  - state transition matrix
%         Q  - process noise covariance matrix 
%         B  - input effect matrix  
%         U  - control input matrix
% ----------------------------------------------------------------------

%% A priori estimate of the current state
% x(t|t-1) = F * x(t-1|t-1)
x = F * x + B * u;
%% A priori estimate of the state covariance matrix
% P(t|t-1) = F * P(t-1|t-1) * F' + Q
P = F * P * F' + Q;
end