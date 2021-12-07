% design PI controller with feedback law u=-kx and integrator

function [Kp, Ki, A_aug, B_aug, C_aug, D_aug] = designPIcontroller()

% get linearized matrices, equilibrium point, and nonlinear functions
[A, B, C, D, X, U, Y, f, g] = linearizeMatrix();

B_red = B(:, 2:5); % B with reduced set of actuators u2:u5
% u1 and u6 are disturbances

% design of integrator
A_int = [[A, [0, 0, 0, 0, 0]']; [0, 0, 0, 1, 0, 0]];
B_int = [B_red; [0, 0, 0, 0]];
C_int = [[C, [0, 0, 0, 0, 0, 0]']];
D_int = D;

% an arbitrary set of poles to place
p = [-10 -10 -10 -1 -500, -10];
[Kp,prec] = place(A,B_int,p); % place poles

% augmented system has 2 dimensional input u_aug for disturbances
% with state equation x_dot = A_aug*x+B_aug*u_aug

B_aug = [[B(:, 1);0], B_int, [B(:, 6);0]]; % B matrix for augmented system
A_aug = A_int-B_aug*Kp; % A matrix for augmented system with feedback law
C_aug = C_int-D_int(:,2:5)*Kp;
D_aug = [D_int(:,1), D_int(:,6)];


end
