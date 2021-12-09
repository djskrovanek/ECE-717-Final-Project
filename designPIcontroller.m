% design PI controller with feedback law u=-kx and integrator

function [Kp, Ki, A_aug, B_aug, C_aug, D_aug] = designPIcontroller()

% get linearized matrices, equilibrium point, and nonlinear functions
[A, B, C, D, X, U, Y, f, g] = linearizeMatrix();

B_red = B(:, 2:5); % B with reduced set of actuators u2:u5
B_com = [B_red; zeros(1,4)];    % added row of zeros
% u1 and u6 are disturbances

J = [0 0 0 1 0];
A_com = [[A, zeros(5,1)]; [J, 0]];  %combined A matrix with J

% an arbitrary set of poles to place
p = [-10 -10 -10 -1 -500 -10];
[K, ~] = place(A_com, B_com, p);
Kp = K(:, 1:5);
Ki = K(:, 6);

D_c = D(:, 2:5);
D_d = [D(:, 1), D(:,6)];

A_aug = [[[A - B_red*Kp], -B_red*Ki]; [J, 0]];
B_aug = [[B(:, 1), B(:, 6)]; zeros(1,2)];
C_aug = [[(C - D_c*Kp)], -D_c*Ki];
D_aug = D_d;

end
