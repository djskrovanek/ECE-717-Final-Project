% design controller with feedback law u=-kx

function [K, A_aug, B_aug, C_aug, D_aug] = designPcontroller()

% get linearized matrices, equilibrium point, and nonlinear functions
[A, B, C, D, X, U, Y, f, g] = linearizeMatrix();

B_red = B(:, 2:5); % B with reduced set of actuators u2:u5
% u1 and u6 are disturbances

% an arbitrary set of poles to place
p = [-10 -10 -10 -1 -500];

[K,prec] = place(A,B_red,p); % place poles

% augmented system has 2 dimensional input u_aug for disturbances
% with state equation x_dot = A_aug*x+B_aug*u_aug

A_aug = A-B_red*K; % A matrix for augmented system with feedback law
B_aug = [B(:, 1), B(:, 6)]; % B matrix for augmented system
C_aug = C-D(:,2:5)*K;
D_aug = [D(:,1), D(:,6)];


end
