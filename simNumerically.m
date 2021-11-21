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

tf1 = 1000; % stop time [sec]

[t_nl1, u_nl1, x_nl1, y_nl1] = simNL(f, g, u1, [t0, tf1], x0);
[t_lti1, u_lti1, x_lti1, y_lti1] = simLTI(A, B, C, D, X, U, Y, u1, [t0 tf1], x0);


figure();
hold on;
plot(t_nl1, getNorm(y_nl1,2), 'DisplayName', 'NL')
plot(t_lti1, getNorm(y_lti1,2), 'DisplayName', 'LTI')
title('Norm of output vs. time', 'Interpreter','latex')
xlabel('Time $(s)$', 'Interpreter','latex')
ylabel('Output norm $||y(t)||_2$', 'Interpreter','latex')
ylim([9.5e5, 9.52e5])
legend('Interpreter','latex')




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

% calculate the p norm of each column of the signal x(t). If x(t) has states
% in row i and times in column j, this returns the p norm of x at each
% time.
function Xnorm = getNorm(x,p)
    Xnorm = zeros(1, size(x,2));
    for i = 1:size(Xnorm,2)
        Xnorm(i) = norm(x(:,i),p);
    end
end

