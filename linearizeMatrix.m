function [A_num, B_num, C_num, D_num] = linearizeMatrix()

%% Defining variables
x = sym('x', [5, 1]);                                                       %state vector
assumeAlso(x, 'real');
u = sym('u', [6, 1]);                                                       %input vector
assumeAlso(u, 'real');

% x matrix parameters
Q_qs = sym('q_qs', 'real');
Q_ds = sym('q_ds', 'real');
Q_fr = sym('q_fr', 'real');
w    = sym('w', 'real');
I_dc = sym('i_dc', 'real');

% y matrix parameters 
V_qs = sym('v_qs', 'real');
V_ds = sym('v_ds', 'real');
V_fr = sym('v_fr', 'real');
P    = sym('P', 'real');

% other parameters and constants 
I_ds = sym('I_ds', 'real');
I_qs = sym('I_qs', 'real');
I_fr = sym('I_fr', 'real');
V_in = sym('V_in', 'real');
Tor_l  = sym('Tor_l', 'real'); % torque
C_qs = sym('C_qs', {'positive', 'real'});
r_qs = sym('r_qs', {'positive', 'real'});
C_lds= sym('C_lds', {'positive', 'real'});
r_lds= sym('r_lds', {'positive', 'real'});
C_lfr= sym('C_lfr', {'positive', 'real'});
r_lfr= sym('r_lfr', {'positive', 'real'});
C_mfs= sym('C_mfs', {'positive', 'real'});
r_mfs= sym('r_mfs', {'positive', 'real'});
J    = sym('J', {'positive', 'real'});
beta = sym('beta', 'real');
L_dc = sym('L_dc', {'positive', 'real'});
r_dc = sym('r_dc', {'positive', 'real'});
N    = sym('N', {'positive', 'real', 'integer'});
z    = sym('z', {'positive', 'real', 'integer'});
M_fe = sym('M_fe', {'positive', 'real'});
M_d  = sym('M_d', {'positive', 'real'});
M_q  = sym('M_q', {'positive', 'real'});
r_ds = sym('r_ds', {'positive', 'real'});
C_fr = sym('C_fr', {'positive', 'real'});
C_ds = sym('C_ds', {'positive', 'real'});
r_fr = sym('r_fr', {'positive', 'real'});

t    = sym('t', {'positive', 'real'});                                      %time variable

tau_1   = sym('tau_1', 'real');
alpha_1 = sym('alpha_1', 'real');
alpha_2 = sym('alpha_2', 'real');
alpha_3 = sym('alpha_3', 'real');
alpha_4 = sym('alpha_4', 'real');
gamma   = sym('gamma', 'real');
% tau_1 = r_qs * C_qs;
% alpha_1 = C_fr / r_ds - C_mfs / r_mfs;
% alpha_2 = C_mfs / r_ds - C_ds / r_mfs;
% alpha_3 = C_mfs / r_fr - C_fr / r_mfs;
% alpha_4 = C_ds / r_fr - C_mfs / r_mfs;
% gamma = 1 / (C_ds * C_fr - C_mfs^2);

% equilibrium point
Q_ds = sym('Q_ds', {'real'});
Q_qs = sym('Q_qs', {'real'});
Q_fr = sym('Q_fr', {'real'});
W = sym('W', {'real'});
I_dc = sym('I_dc', {'real'});

Tor_l = sym('Tor_l', {'real'});
I_fr = sym('I_fr', {'real'});
M_fe = sym('M_fe', {'real'});
M_q = sym('M_q', {'real'});
M_d = sym('M_d', {'real'});
V_in = sym('V_in', {'real'});

X_1 = sym('X_1', {'real'});
X_2 = sym('X_2', {'real'});
X_3 = sym('X_3', {'real'});
X_4 = sym('X_4', {'real'});
X_5 = sym('X_5', {'real'});

U_1 = sym('U_1', {'real'});
U_2 = sym('U_2', {'real'});
U_3 = sym('U_3', {'real'});
U_4 = sym('U_4', {'real'});
U_5 = sym('U_5', {'real'});
U_6 = sym('U_6', {'real'});

tableParams = CalcEquilibrium();
vars = {'V_ds'; 'V_qs'; 'V_fr'; 'We'; 'W'; 'I_dc'; 'V_in'; 'C_lds'; 'C_lfr'; ...
    'C_mfs'; 'C_qs'; 'z'; 'L_dc'; 'r_dc'; 'J'; 'beta'; 'N'; 'r_lds';...
    'r_lfr'; 'r_mfs'; 'r_qs'; 'C_ds'; 'C_fr'; 'r_ds'; 'r_fr'; 'tau_1'; 'alpha_1';...
    'alpha_2'; 'alpha_3'; 'alpha_4'; 'gamma'; 'Q_qs'; 'Q_ds'; 'Q_fr'; 'I_fr';...
    'Tor_l'; 'I_qs'; 'I_ds'; 'M_q'; 'M_d'; 'M_fe'; 'P_m'; 'P'; 'Eta'};
values = cell(length(vars), 1);
for i = 1:length(vars)
    for j = 1:length(tableParams.vals)
        if string(vars(i)) == string(tableParams.vars(j))
            values(i) = table2cell(tableParams(j,2)); 
            continue
        end
   end
end

% values = {-10, -10, 10, 100e3, 0.67e6, 512e-9, 502e3, ...
%     430e-9, 82e3, 128e-9, 15e6, 82e-9, 420e3, 3.1e6, 10, 12, 12, 10, ...
%     96, -0.1, -0.1, -0.1, 502e3, 210e-9, 512e-9, 408.6e3, ...
%     100e3, 0, 175e3, 1e6, 1.5};
%valueTable = table([string(vars); (values)].');                             %click in Workspace to more easily read parameter assignments

%% System dynamics and output 
f = [u(4)*x(5) - z*x(2)*x(4) - x(1)/tau_1;
    u(5)*x(5) + z*x(1)*x(4) - gamma*alpha_1*x(2) - gamma*alpha_2*x(3);
    u(2) - gamma*alpha_3*x(2) - gamma*alpha_4*x(3);
    -beta*x(4)/J + u(1)/J + x(1)*x(2)*(3*z/(2*J*C_qs)-3*z*gamma*C_fr/(2*J)) - 3*z*gamma*C_mfs/(2*J)*x(1)*x(3);
    N*u(3)*u(6)/L_dc - r_dc*x(5)/L_dc - 3*u(4)*x(1)/(2*L_dc*C_qs) - 3*C_fr*gamma*u(5)*x(2)/(2*L_dc) - 3*C_mfs*gamma*u(5)*x(3)/(2*L_dc)
    ];

g = [x(1) / C_qs;
    gamma * C_fr * x(2) + gamma * C_mfs * x(3);
    gamma * C_mfs * x(2) + gamma * C_ds * x(3);
    x(4);
    x(5);
    u(3) * N * u(6) * x(5)
    ];

%{
g = [q_qs / C_qs;
    gamma * C_fr * q_ds + gamma * C_mfs * q_fr;
    gamma * C_mfs * q_ds + gamma * C_ds * q_fr;
    w;
    i_dc;
    m_fe * N * v_in * i_dc
    ];
%}
    
%y = [v_qs; v_ds; v_fr; w; i_dc; P];

%% Check if the solution solves the ODE
xsol = ...
    [X_1;
    X_2;
    X_3;
    X_4;
    X_5;
    ];

% components of xsol in terms of physical symbolic variables
xsol_vars = ...
    [Q_qs;
    Q_ds;
    Q_fr;
    W;
    I_dc;
    ];

xsol_num = double(subs(xsol_vars, vars, values));
%{
xsol = ...
    [C_qs * v_qs;
    C_ds * v_ds - C_mfs * v_fr;
    -C_mfs * v_ds + C_fr * v_fr;
    w;
    m_fe * N * v_in - 3/2 * m_q * v_qs / r_dc % note: this equation has an error since units don't match, volts - amps
    ];
%}

usol = ...
    [U_1;
    U_2;
    U_3;
    U_4;
    U_5;
    U_6;
    ];

% components of xsol in terms of physical symbolic variables
usol_vars = ...
    [Tor_l;
    I_fr;
    M_fe;
    M_q;
    M_d;
    V_in
    ];

usol_num = double(subs(usol_vars, vars, values));

dxdt = subs(f, [x;u], [xsol;usol]);
if all(dxdt==jacobian(xsol,t))
    disp('xsol solves the ODE for usol');
else
    disp('xsol does NOT solve the ODE for usol');
end

dxdt_num = subs(subs(f, [x;u], [xsol_num; usol_num]), vars, values);
error_num = dxdt_num-jacobian(xsol, t); % numerical error, should be near 0
epsilon = 1e-9; % some tolerance for numerical errors
if all(error_num < epsilon)
    disp('xsol_num solves the ODE for usol');
else
    disp('xsol_num does NOT solve the ODE for usol');
end

%% Compute the linearization
As = jacobian(f, x);
Bs = jacobian(f, u);
Cs = jacobian(g, x);
Ds = jacobian(g, u);

% matrices in terms of symbolic parameters
A = subs(As, [x;u], [xsol;usol]);
B = subs(Bs, [x;u], [xsol;usol]);
C = subs(Cs, [x;u], [xsol;usol]);
D = subs(Ds, [x;u], [xsol;usol]);

% matrices with numerical values substituted
A_num = double(subs(A, [vars; xsol; usol], [cell2mat(values); xsol_num; usol_num]));
B_num = double(subs(B, [vars; xsol; usol], [cell2mat(values); xsol_num; usol_num]));
C_num = double(subs(C, [vars; xsol; usol], [cell2mat(values); xsol_num; usol_num]));
D_num = double(subs(D, [vars; xsol; usol], [cell2mat(values); xsol_num; usol_num]));

end