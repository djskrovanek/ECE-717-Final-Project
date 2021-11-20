% simulate LTI system with given parameters
% inputs:
% A, B, C, D: matrices
% X, U, Y: equilibrium point values of x, u, and y
% u: input as a function handle of time u(t)
% t: either [t0, tf] with auto step or a row vector of time points to use
% x0: initial state
function [t_lti, u_lti, x_lti, y_lti] = simLTI(A, B, C, D, X, U, Y, u, t, x0)
    % Note: _lin subscript denotes small signal (linearized) quantities,
    % e.g. x_lin = \tilde(x) = x-X.
    % whereas _lti subscript denotes actual signal.
    % e.g. x_lti = x

    % compute small signal linearized values from actual values
    x0_lin = x0-X;
    u_lin = @(t) u(t)-U;
    
    %[t_lti, x_lin] = ode45(@(t,x_lin) A*x_lin+B*u_lin(t), t,x0_lin); % non-stiff solver
    [t_lti, x_lin] = ode15s(@(t,x_lin) A*x_lin+B*u_lin(t), t,x0_lin); % stiff solver
    t_lti = t_lti'; % t(1,i) is time at time i
    x_lin = x_lin';
    x_lti = x_lin+X; % x(i,j) is state i at time j
    
    u_lti = u(t_lti); % u as a vector vs time
    
    y_lin = C*x_lin+D*u_lin(t_lti); 
    y_lti = y_lin+Y; % y(i,j) is output i at time j
end