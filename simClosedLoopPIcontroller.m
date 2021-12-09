simulates closed-loop (with PI controller) response of LTI and NL system

clear all; close all; clc;

[params] = CalcEquilibrium; % fill workspace with parameters
for i = 1:height(params)
    assignin('base', string(table2array(params(i,1))), double(table2array(params(i,2))));
end

[~, u_B, x_B, y_B] = CalcBaseVals(); % get base values for signals

get linearized matrices, equilibrium point, and nonlinear functions
[A, B, C, D, X, U, Y, f, g] = linearizeMatrix();

[Kp, Ki, A_aug, B_aug, C_aug, D_aug] = designPIcontroller();
U_aug = [U(1); U(6)];

% Set simulation parameters

t0 = 0; % initial time [sec]
% Note: time step is variable and set by ode45()

% Note: specify input as a function of time

x0 = X; % initial state

% simulate a step increase in torque to 5% beyond rated value

u2_aug = @(t) [U(1)*1.05; U(6)]*ones(size(t)); % step in torque at t = 0

tf2 = 30; % stop time [sec]

f(x,u) for augmented NL system with controller. New input u_aug has dimension 2 x length(t).
Using same feedback law \tilde{u}=-k*\tilde{x}. note controller acts on the error \tilde{x}=x-X, not just x.
f_aug = @(x,u_aug) f(x, [u_aug(1,:); U(2:5)-K*(x-X); u_aug(2,:)]);
g(x,u) for augmented NL system with controller. New input u_aug has dimension 2 x length(t).
g_aug = @(x,u_aug) g(x, [u_aug(1,:); U(2:5)-K*(x-X); u_aug(2,:)]);

[t_nl2, u_aug_nl2, x_nl2, y_nl2] = simNL(f_aug, g_aug, u2_aug, [t0, tf2], x0);
[t_lti2, u_aug_lti2, x_lti2, y_lti2] = simLTI(A_aug, B_aug, C_aug, D_aug, X, U_aug, Y, u2_aug, [t0 tf2], x0);

calculate the full input vector u with 6 inputs from u_aug
u_nl2 = [u_aug_nl2(1,:); U(2:5)-K*(x_nl2-X); u_aug_nl2(2,:)];
u_lti2 = [u_aug_lti2(1,:); U(2:5)-K*(x_lti2-X); u_aug_lti2(2,:)];

figure();
hold on;
plot(t_nl2, y_nl2(4,:), 'DisplayName', 'NL')
plot(t_lti2, y_lti2(4,:), 'DisplayName', 'LTI')
title('Shaft speed vs time with torque step')
xlabel('Time $t$ (s)', 'Interpreter', 'latex')
ylabel('Speed $\omega$ (rad/s)', 'Interpreter', 'latex')
ylim([0, 2*Y(6)])
legend()

figure();
hold on;
plot(t_nl2, y_nl2(6,:)*1e-6, 'DisplayName', 'NL')
plot(t_lti2, y_lti2(6,:)*1e-6, 'DisplayName', 'LTI')
title('Output power vs time with torque step')
xlabel('Time (s)', 'Interpreter', 'latex')
ylabel('Output power $p_{out}$ (MW)', 'Interpreter', 'latex')
ylim([0, 2*Y(6)])
legend()

calculate normalized (pu) outputs and states

X_pu = X./x_B;
Y_pu = Y./y_B;
U_pu = U./u_B;

yn_lti2 = y_lti2./y_B;
xn_lti2 = x_lti2./x_B;
un_lti2 = u_lti2./u_B;
yn_nl2 = y_nl2./y_B;
xn_nl2 = x_nl2./x_B;
un_nl2 = u_nl2./u_B;


plot all states, outputs, and inputs vs time
figure();
yLabels = ["x1", "x2", "x3", "x4", "x5"];
for i = 1:length(X)
    ax = subplot(3,2, i);
    hold on;
    plot(t_lti2, xn_lti2(i,:), 'DisplayName', 'LTI');
    plot(t_nl2, xn_nl2(i,:), 'DisplayName', 'NL');
    plot(t_lti2, X_pu(i).*ones(size(t_lti2)), '--', 'DisplayName', 'Equilibrium')
    ylabel(yLabels(i), 'Interpreter', 'latex')
    xlabel('Time (s)', 'Interpreter', 'latex')
    set(gca, 'YLimSpec', 'padded');
    if (i ==5)
        ax.Position(1) = 0.5-ax.Position(3)/2;
    end
    legend('Location', 'Southwest')
end
sgtitle('PU states vs time with torque step')

figure();
yLabels = ["y1", "y2", "y3", "y4", "y5", "y6"];
for i = 1:length(Y)
    subplot(3,2, i)
    plot(t_lti2, yn_lti2(i,:), 'DisplayName', 'LTI');
    hold on;
    plot(t_nl2, yn_nl2(i,:), 'DisplayName', 'NL');
    plot(t_lti2, Y_pu(i).*ones(size(t_lti2)), '--', 'DisplayName', 'Equilibrium')
    set(gca, 'YLimSpec', 'padded');
    ylabel(yLabels(i), 'Interpreter', 'latex')
    xlabel('Time (s)', 'Interpreter', 'latex')
    legend('Location', 'Southwest')
end
sgtitle('PU outputs vs time with torque step')
xlabel('Time (s)', 'Interpreter', 'latex')

figure();
yLabels = ["u1", "u2", "u3", "u4", "u5", "u6"];
for i = 1:6
    subplot(3,2, i)
    plot(t_lti2, un_lti2(i,:), 'DisplayName', 'LTI');
    hold on;
    plot(t_nl2, un_nl2(i,:), 'DisplayName', 'NL');
    plot(t_lti2, U_pu(i).*ones(size(t_lti2)), '--', 'DisplayName', 'Equilibrium')
    set(gca, 'YLimSpec', 'padded');
    ylabel(yLabels(i), 'Interpreter', 'latex')
    xlabel('Time (s)', 'Interpreter', 'latex')
    legend('Location', 'Southwest')
end
sgtitle('PU inputs vs time with torque step')
xlabel('Time (s)', 'Interpreter', 'latex')

% calculate final pu norm
Xnorm_lti = norm(xn_lti2(:,end),2)
Xnorm_nl = norm(xn_nl2(:,end),2)
Ynorm_lti = norm(yn_lti2(:,end),2)
Ynorm_nl = norm(yn_nl2(:,end),2)

normalize error between norms to a percent
Xerror_lti = norm((xn_lti2(:,end)-X_pu)/X_pu,2)
Xerror_nl = norm((xn_nl2(:,end)-X_pu)/X_pu,2)
Yerror_lti = norm((yn_lti2(:,end)-Y_pu)/Y_pu,2)
Yerror_nl = norm((yn_nl2(:,end)-Y_pu)/Y_pu,2)

output gain matrix K to Latex for report
Kdisp = vpa(sym(K),3);
latex(Kdisp)