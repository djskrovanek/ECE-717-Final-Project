clear
clc

%% Defining variables
x = sym('x', [5, 1]);                                                       %state vector
assumeAlso(x, 'real');
u = sym('u', [6, 1]);                                                       %input vector
assumeAlso(u, 'real');

% x matrix parameters
Q_qs = sym('Q_qs', 'real');
Q_ds = sym('Q_ds', 'real');
Q_fr = sym('Q_fr', 'real');
w    = sym('w', 'real');
i_dc = sym('i_dc', 'real');

% y matrix parameters 
v_qs = sym('v_qs', 'real');
v_ds = sym('v_ds', 'real');
v_fr = sym('v_fr', 'real');
P    = sym('P', 'real');

% other parameters and constants 
i_ds = sym('i_ds', 'real');
i_qs = sym('i_qs', 'real');
i_fr = sym('i_fr', 'real');
v_in = sym('v_in', 'real');
T_l  = sym('T_l', 'real');
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
m_fe = sym('m_fe', {'positive', 'real'});
m_d  = sym('m_d', {'positive', 'real'});
m_q  = sym('m_q', {'positive', 'real'});
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

vars = {i_ds, i_qs, i_fr, v_in, T_l, C_qs, r_qs,...
    C_lds, r_lds, C_lfr, r_lfr, C_mfs, r_mfs, J, beta, L_dc, r_dc, N,...
    z, m_fe, m_d, m_q, r_ds, C_fr, C_ds, r_fr, ...
    v_qs, v_ds, v_fr, P, w};
values = {-10, -10, 10, 100e3, 0.67e6, 512e-9, 502e3, ...
    430e-9, 82e3, 128e-9, 15e6, 82e-9, 420e3, 3.1e6, 10, 12, 12, 10, ...
    96, -0.1, -0.1, -0.1, 502e3, 210e-9, 512e-9, 408.6e3, ...
    100e3, 0, 175e3, 1e6, 1.5};
valueTable = table([string(vars); (values)].');                             %click in Workspace to more easily read parameter assignments

%% System dynamics and output 
f = [u(4)*x(5) - z/2*x(2)*x(4) - x(1)/tau_1;
    u(5)*x(5) + z/2*x(1)*x(4) - gamma*alpha_1*x(2) - gamma*alpha_2*x(3);
    u(2) - gamma*alpha_3*x(2) + gamma*alpha_4*x(3);
    -beta*x(4)/J + u(1)/J + x(1)*x(2)*(3*z/(2*J*C_qs)-3*z*gamma*C_fr/(2*J)) - 3*z*gamma*C_mfs/(2*J)*x(1)*x(3);
    N*u(3)*u(6)/L_dc - r_dc*x(5)/L_dc - 3*u(4)*x(1)/(2*L_dc*C_qs) - 3*C_fr*gamma*u(5)*x(2)/(2*L_dc) - 3*C_mfs*gamma*u(5)*x(3)/(2*L_dc)
    ];

g = [Q_qs / C_qs;
    gamma * C_fr * Q_ds + gamma * C_mfs * Q_fr;
    gamma * C_mfs * Q_ds + gamma * C_ds * Q_fr;
    w;
    i_dc;
    m_fe * N * v_in * i_dc
    ];
y = [v_qs; v_ds; v_fr; w; i_dc; P];

%% Check if the solution solves the ODE
xsol = ...
    [C_qs * v_qs;
    C_ds * v_ds - C_mfs * v_fr;
    -C_mfs * v_ds + C_fr * v_fr;
    w;
    m_fe * N * v_in - 3/2 * m_q * v_qs / r_dc % note: this equation has an error since units don't match, volts - amps
    ];
xsol_num = vpa(subs(xsol, vars, values));

usol = ...
    [T_l;
    i_fr;
    m_fe;
    m_q;
    m_d;
    v_in
    ];
usol_num = vpa(subs(usol, vars, values));

dxdt = subs(f, [x;u], [xsol;usol]);
if all(dxdt==jacobian(xsol,t))
    disp('xsol solves the ODE for usol');
else
    disp('xsol does NOT solve the ODE for usol');
end

dxdt_num = subs(subs(f, [x;u], [xsol_num; usol_num]), vars, values);
if all(dxdt_num==jacobian(xsol, t))
    disp('xsol_num solves the ODE for usol');
else
    disp('xsol_num does NOT solve the ODE for usol');
end

%% Compute the linearization
As = jacobian(f, x);
Bs = jacobian(f, u);
Cs = jacobian(g, x);
Ds = jacobian(g, u);

A = subs(As, [x;u], [xsol;usol]);
B = subs(Bs, [x;u], [xsol;usol]);
C = subs(Cs, [x;u], [xsol;usol]);
D = subs(Ds, [x;u], [xsol;usol]);
