clear all; close all; clc;

[A, B, C, D, ~, ~, ~, ~, ~] = linearizeMatrix(); % numerical matrices

n = size(A,1); % size of state space


%% check properties of open loop linearized system

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
