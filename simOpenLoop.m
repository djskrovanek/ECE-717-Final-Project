% simulates open-loop response of LTI and NL system to given input

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
plot(t_lti1, getNorm(x_lti1./x_B,2), 'DisplayName', 'x: LTI')
plot(t_nl1, getNorm(x_nl1./x_B,2), 'DisplayName', 'x: NL')
plot(t_lti1, getNorm(y_lti1./y_B,2), 'DisplayName', 'y: LTI')
plot(t_nl1, getNorm(y_nl1./y_B,2), 'DisplayName', 'y: NL')
%title('Norm of PU state & output vs. time')%, 'Interpreter','latex')
xlabel('Time $(s)$', 'Interpreter','latex')
ylabel('Norm (PU)', 'Interpreter','latex')
%ylim([9.5e5, 9.52e5])
legend('Interpreter','latex','location','east')
set(gca, 'YLimSpec', 'padded');
exportgraphics(gcf,'Output norm equilibrium.jpg','Resolution',300)





%% simulate a step increase in torque to 5% beyond rated value

u2 = @(t) [U(1)*1.05; U(2:6)]*ones(size(t)); % step in torque at t = 0

tf2 = 1000; % stop time [sec]

[t_nl2, u_nl2, x_nl2, y_nl2] = simNL(f, g, u2, [t0, tf2], x0);
[t_lti2, u_lti2, x_lti2, y_lti2] = simLTI(A, B, C, D, X, U, Y, u2, [t0 tf2], x0);

% calculate normalized (pu) outputs and states

X_pu = X./x_B;
Y_pu = Y./y_B;
U_pu = U./u_B;

yn_lti2 = y_lti2./y_B;
xn_lti2 = x_lti2./x_B;
yn_nl2 = y_nl2./y_B;
xn_nl2 = x_nl2./x_B;

figure();
hold on;
plot(t_nl2, y_nl2(4,:), 'DisplayName', 'NL')
plot(t_lti2, y_lti2(4,:), 'DisplayName', 'LTI')
plot(t_nl2, Y(4).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
title('Shaft speed vs time with torque step')
xlabel('Time $t$ (s)', 'Interpreter', 'latex')
ylabel('Speed $\omega$ (rad/s)', 'Interpreter', 'latex')
%ylim([0, 2*Y(6)])
legend()

figure();
hold on;
plot(t_nl2, y_nl2(6,:)*1e-6, 'DisplayName', 'NL')
plot(t_lti2, y_lti2(6,:)*1e-6, 'DisplayName', 'LTI')
plot(t_nl2, Y(6)*1e-6.*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
title('Output power vs time with torque step')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Output power $p_{out}$ (MW)', 'Interpreter', 'latex')
%ylim([0, 2*Y(6)])
legend()


% plot all states and outputs vs time
% figure();
% yLabels = ["x1", "x2", "x3", "x4", "x5"];
% for i = 1:length(X)
%     subplot(3,2, i)
%     plot(t_lti2, xn_lti2(i,:), 'DisplayName', 'LTI');
%     hold on;
%     plot(t_nl2, xn_nl2(i,:), 'DisplayName', 'NL');
%     ylabel(yLabels(i), 'Interpreter', 'latex')
%     %legend('Location', 'Southwest')
% end
% subplot(3,2,6)
% sgtitle('PU states vs time with torque step')
% xlabel('Time (s)', 'Interpreter', 'latex')

%plot and tweak states vs time
yLabels = ["$x_1$ (pu)", "$x_2$ (pu)", "$x_3$ (pu)", "$x_4$ (pu)", "$x_5$ (pu)"];

figure
subplot(3,2, 1)
plot(t_lti2, xn_lti2(1,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, xn_nl2(1,:), 'DisplayName', 'NL');
plot(t_nl2, X_pu(1).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(1), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

subplot(3,2, 2)
plot(t_lti2, xn_lti2(2,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, xn_nl2(2,:), 'DisplayName', 'NL');
plot(t_nl2, X_pu(2).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(2), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

subplot(3,2, 3)
plot(t_lti2, xn_lti2(3,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, xn_nl2(3,:), 'DisplayName', 'NL');
plot(t_nl2, X_pu(3).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(3), 'Interpreter', 'latex')
ylim([2.43, 2.45])
xlabel('Time (s)', 'Interpreter', 'latex')

subplot(3,2, 4)
plot(t_lti2, xn_lti2(4,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, xn_nl2(4,:), 'DisplayName', 'NL');
plot(t_nl2, X_pu(4).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(4), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

ax = subplot(3,2, 5);
plot(t_lti2, xn_lti2(5,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, xn_nl2(5,:), 'DisplayName', 'NL');
plot(t_nl2, X_pu(5).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ax.Position(1) = 0.5-ax.Position(3)/2;
ylabel(yLabels(5), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

%sgtitle('PU states vs time with torque step')
exportgraphics(gcf,'PU states vs time with torque step.jpg','Resolution',300)

%xlabel('Time (s)', 'Interpreter', 'latex')
% subplot(3,2,6)
% plot(0,0, 'DisplayName', 'LTI')
% hold on
% plot(0,0, 'DisplayName', 'NL')
% set(gca, 'visible', 'off');
% legend('FontSize', 20)


% figure();
% yLabels = ["y1", "y2", "y3", "y4", "y5", "y6"];
% for i = 1:length(Y)
%     subplot(length(Y),1, i)
%     plot(t_lti2, yn_lti2(i,:), 'DisplayName', 'LTI');
%     hold on;
%     plot(t_nl2, yn_nl2(i,:), 'DisplayName', 'NL');
%     ylabel(yLabels(i), 'Interpreter', 'latex')
%     legend('Location', 'Southwest')
% end
% sgtitle('PU outputs vs time with torque step')
% xlabel('Time (s)', 'Interpreter', 'latex')

%plot and tweak states vs time
yLabels = ["$y_1$", "$y_2$", "$y_3$", "$y_4$", "$y_5$", "$y_6$"];

figure
subplot(3,2, 1)
plot(t_lti2, yn_lti2(1,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, yn_nl2(1,:), 'DisplayName', 'NL');
plot(t_nl2, Y_pu(1).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(1), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

subplot(3,2, 2)
plot(t_lti2, yn_lti2(2,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, yn_nl2(2,:), 'DisplayName', 'NL');
plot(t_nl2, Y_pu(2).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(2), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

subplot(3,2, 3)
plot(t_lti2, yn_lti2(3,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, yn_nl2(3,:), 'DisplayName', 'NL');
plot(t_nl2, Y_pu(3).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(3), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

subplot(3,2, 4)
plot(t_lti2, yn_lti2(4,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, yn_nl2(4,:), 'DisplayName', 'NL');
plot(t_nl2, Y_pu(4).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(4), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

subplot(3,2, 5)
plot(t_lti2, yn_lti2(5,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, yn_nl2(5,:), 'DisplayName', 'NL');
plot(t_nl2, Y_pu(5).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(5), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')


subplot(3,2, 6)
plot(t_lti2, yn_lti2(6,:), 'DisplayName', 'LTI');
hold on;
plot(t_nl2, yn_nl2(6,:), 'DisplayName', 'NL');
plot(t_nl2, Y_pu(6).*ones(size(t_nl2)), '--', 'DisplayName', 'Equilibrium')
ylabel(yLabels(6), 'Interpreter', 'latex')
xlabel('Time (s)', 'Interpreter', 'latex')

%sgtitle('PU outputs vs time with torque step')
exportgraphics(gcf,'PU outputs vs time with torque step.jpg','Resolution',300)

%% calculate final pu norm
Xnorm_lti = norm(xn_lti2(:,end),2)
Xnorm_nl = norm(xn_nl2(:,end),2)
Ynorm_lti = norm(yn_lti2(:,end),2)
Ynorm_nl = norm(yn_nl2(:,end),2)

% normalize error between pu norms to a percent
Xerror_lti = norm((xn_lti2(:,end)-X_pu)/X_pu,2)
Xerror_nl = norm((xn_nl2(:,end)-X_pu)/X_pu,2)
Yerror_lti = norm((yn_lti2(:,end)-Y_pu)/Y_pu,2)
Yerror_nl = norm((yn_nl2(:,end)-Y_pu)/Y_pu,2)


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

