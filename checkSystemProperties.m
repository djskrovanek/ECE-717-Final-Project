clear all; close all; clc;

[A, B, C, D, ~, ~, ~, ~, ~] = linearizeMatrix(); % numerical matrices

n = size(A,1); % size of state space


%% check properties of open loop linearized system
% Note: this controllability calculation assumes we have control of all 5
% inputs. Really u2-u5 are the only ones we can actuate, and u1 (the load torque) and
% u6 (the dc grid voltage) are disturbances, not actuators.

[eigvecs, d] = eigs(A); % matrix of eigenvectors and diagonal matrix of eigenvalues
eigvals = diag(d) % vector of eigenvalues

% check stability
if (real(eigvals) < zeros(size(eigvals)))
    disp('Open-loop system is asymptotically stable.')
else
    disp('Open-loop system is not asymptotically stable.')
end

% check controllability (also guarantees reachability)
C_ctr = buildCctr(A,B);
if (rank(C_ctr) == n)
    disp('Open-loop system is controllable.')
else
    disp('Open-loop system is not controllable.')
end

% check observability
O = buildO(A,C);
if (rank(O) == n)
    disp('Open-loop system is observable.')
else
    disp('Open-loop system is not observable.')
end

%% check BIBO stability
for i = 1:n
    [~, den] = ss2tf(A,B,C,D, i);
    P = roots(den); %compute poles of transfer function
    stable = any(P>0);
    if stable == 1
        disp(['Not BIBO stable for', num2str(i), ' input'])
    end
end

%% check properties of a reduced system which only has 4 inputs/actuators
% wind speed and dc grid voltage are treated as disturbances

% stability/observability results are the same, but check controllability 
B_red = B(:, 2:5); % the only actuators here are u2:u5

% check controllability (also guarantees reachability)
C_ctr_red = buildCctr(A,B_red);
if (rank(C_ctr) == n)
    disp('Reduced system is controllable.')
else
    disp('Reduced system is not controllable.')
end

%% helper functions

% build controllability matrix
function C_ctr = buildCctr(A,B)
    C_ctr = [B];
    for k = 1:(size(A,1)-1)
        C_ctr = [C_ctr, A^k*B];
    end
end

% build observability matrix
function O = buildO(A,C)
    O = [C];
    for k = 1:(size(A,1)-1)
        O = [O; C*A^k];
    end
end
