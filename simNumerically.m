clear all; close all; clc;

[params] = CalcEquilibrium; % fill workspace with parameters
for i = 1:height(params)
    assignin('base', string(table2array(params(i,1))), double(table2array(params(i,2))));
end

[~, u_B, x_B, y_B] = CalcBaseVals(); % get base values for signals

% get linearized matrices, equilibrium point, and nonlinear functions
[A, B, C, D, X, U, Y, f, g] = linearizeMatrix();

%% Set simulation parameters

t0 = 0; % initial time [sec]
% Note: time step is variable and set by ode45()

% Note: specify input as a function of time

x0 = X; % initial state

%% simulate systems at equilibrium point

u1 = @(t) U*ones(size(t)); % constant input at equilibrium value

tf1 = 100; % stop time [sec]

[t_nl1, u_nl1, x_nl1, y_nl1] = simNL(f, g, u1, [t0, tf1], x0);
[t_lti1, u_lti1, x_lti1, y_lti1] = simLTI(A, B, C, D, X, U, Y, u1, [t0 tf1], x0);

figure();
hold on;
plot(t_nl1, y_nl1(6,:)*1e-6, 'DisplayName', 'NL')
plot(t_lti1, y_lti1(6,:)*1e-6, 'DisplayName', 'LTI')
title('Output power vs time at equilibrium point')
xlabel('Time (s)')
ylabel('Output power (MW)')
%ylim([0, 2*Y(6)])
legend()




%% simulate a step increase in torque to 0.5% beyond rated value

u2 = @(t) [U(1)*1.005; U(2:6)]*ones(size(t)); % step in torque at t = 0

tf2 = 1000; % stop time [sec]

[t_nl2, u_nl2, x_nl2, y_nl2] = simNL(f, g, u2, [t0, tf2], x0);
[t_lti2, u_lti2, x_lti2, y_lti2] = simLTI(A, B, C, D, X, U, Y, u2, [t0 tf2], x0);

figure();
hold on;
plot(t_nl2, y_nl2(4,:), 'DisplayName', 'NL')
plot(t_lti2, y_lti2(4,:), 'DisplayName', 'LTI')
title('Shaft speed vs time with torque step')
xlabel('Time $t$ (s)', 'Interpreter', 'latex')
ylabel('Speed $\omega$ (rad/s)', 'Interpreter', 'latex')
%ylim([0, 2*Y(6)])
legend()

figure();
hold on;
plot(t_nl2, y_nl2(6,:)*1e-6, 'DisplayName', 'NL')
plot(t_lti2, y_lti2(6,:)*1e-6, 'DisplayName', 'LTI')
title('Output power vs time with torque step')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Output power $p_{out}$ (MW)', 'Interpreter', 'latex')
%ylim([0, 2*Y(6)])
legend()

% calculate normalized (pu) outputs and states

X_pu = X./x_B;
Y_pu = Y./y_B;
U_pu = U./u_B;

yn_lti2 = y_lti2./y_B;
xn_lti2 = x_lti2./x_B;
yn_nl2 = y_nl2./y_B;
xn_nl2 = x_nl2./x_B;


% plot all states and outputs vs time
figure();
yLabels = ["x1", "x2", "x3", "x4", "x5"];
for i = 1:length(X)
    subplot(length(X),1, i)
    plot(t_lti2, xn_lti2(i,:), 'DisplayName', 'LTI');
    hold on;
    plot(t_nl2, xn_nl2(i,:), 'DisplayName', 'NL');
    ylabel(yLabels(i), 'Interpreter', 'latex')
    legend('Location', 'Southwest')
end
sgtitle('PU states vs time with torque step')
xlabel('Time (s)', 'Interpreter', 'latex')

figure();
yLabels = ["y1", "y2", "y3", "y4", "y5", "y6"];
for i = 1:length(Y)
    subplot(length(Y),1, i)
    plot(t_lti2, yn_lti2(i,:), 'DisplayName', 'LTI');
    hold on;
    plot(t_nl2, yn_nl2(i,:), 'DisplayName', 'NL');
    ylabel(yLabels(i), 'Interpreter', 'latex')
    legend('Location', 'Southwest')
end
sgtitle('PU outputs vs time with torque step')
xlabel('Time (s)', 'Interpreter', 'latex')



%% simulate a step increase in Mfe to 1% beyond rated value

%{
u3 = @(t) [U(1:2); 1.01*U(3); U(4:6)]*ones(size(t)); % step in m_fe from rated to 101% at t = 0

tf3 = 600; % stop time [sec]

[t_nl3, u_nl3, x_nl3, y_nl3] = simNL(f, g, u3, [t0, tf3], x0);
[t_lti3, u_lti3, x_lti3, y_lti3] = simLTI(A, B, C, D, X, U, Y, u2, [t0 tf3], x0);

figure();
hold on;
plot(t_nl3, y_nl3(6,:)*1e-6, 'DisplayName', 'NL')
plot(t_lti3, y_lti3(6,:)*1e-6, 'DisplayName', 'LTI')
title('Output power vs time with $m_{fe}$ step', 'Interpreter', 'latex')
xlabel('Time (s)')
ylabel('Output power (MW)')
%ylim([0, 2*Y(6)])
legend()
%}




%% helper functions

% simulate NL system with given parameters
% inputs:
% f, g: function handles for f(x,u) and g(x,u)
% u: input as a function handle of time u(t)
% t: either [t0, tf] with auto step or a row vector of time points to use
% x0: initial state
function [t_nl, u_nl, x_nl, y_nl] = simNL(f, g, u, t, x0)
    opts = odeset('RelTol',1e-4, 'Refine', 4); % solver settings
    %[t_nl, x_nl] = ode45(@(t,x) f(x,u(t)),t,x0); % non-stiff solver
    [t_nl, x_nl] = ode15s(@(t,x) f(x,u(t)),t,x0, opts); % stiff solver
    %[t_nl, x_nl] = ode23s(@(t,x) f(x,u(t)),t,x0); % stiff solver
    t_nl = t_nl'; % % t(1,i) is time at time i
    x_nl = x_nl'; % x(i,j) is state i at time j
    u_nl = u(t_nl); % u as a vector vs time
    y_nl = g(x_nl, u_nl); % y(i,j) is output i at time j
end

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