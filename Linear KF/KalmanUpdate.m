function [x, P]= KalmanUpdate(x, P, H, R)
% ----------------------------------------------------------------------
%                           Kalman Filter Update 
%
%   This function implements the kalman filter update. 
%  
%   Usage:  [x, P]= KalmanUpdate(x, P, H, R)
%           
%         x  - state vector
%         P  - covariance of the system state 
%         H  - measurement (sensor) mosel matrix
%         R  - measurement noise covariance matrix 
% ----------------------------------------------------------------------

y = z - H * x;          % Innovation or measurement pre-fit residual         
S = H * P * H' + R;     % Innovation (or pre-fit residual) covariance
K = P * Ht * inv(S);    % Optimal Kalman gain
%% A posteriori (updated) estimate of the current state 
% x(t|t) = x(t|t-1) + K(t)*(y(t)-y(t|t-1)
x = x + K * y;
%% A posteriori (updated) state covariance matrix 
% P(t|t) = (I - K(t)*C) * P(t|t-1) 
P = (eye(length(x))-K*H) * P;
end