clear
clc

%% Defining variables
x = sym('x', [5, 1]);                                                       %state vector
assumeAlso(x, 'real');
u = sym('u', [6, 1]);                                                       %input vector
assumeAlso(u, 'real');

i_ds = sym('i_ds', 'real');
i_qs = sym('i_qs', 'real');
v_fr = sym('v_fr', 'real');
i_fr = sym('i_fr', 'real');
v_ds = sym('v_ds', 'real');
v_qs = sym('v_qs', 'real');
Q_fr = sym('Q_fr', 'real');
Q_ds = sym('Q_ds', 'real');
Q_qs = sym('Q_qs', 'real');
i_dc = sym('i_dc', 'real');
v_in = sym('v_in', 'real');
w    = sym('w', 'real');
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
p    = sym('p', {'positive', 'real', 'integer'});
m_fe = sym('m_fe', {'positive', 'real'});
m_d  = sym('m_d', {'positive', 'real'});
m_q  = sym('m_q', {'positive', 'real'});
r_ds = sym('r_ds', {'positive', 'real'});
C_fr = sym('C_fr', {'positive', 'real'});
C_ds = sym('C_ds', {'positive', 'real'});
r_fr = sym('r_fr', {'positive', 'real'});
t    = sym('t', {'positive', 'real'});                                      %time variable

tau_1 = r_qs * C_qs;
tau_2 = r_ds * C_fr - r_mfs * C_mfs;
tau_3 = r_ds * C_mfs - r_mfs * C_ds;
tau_4 = r_fr * C_mfs - r_mfs * C_fr;
tau_5 = r_fr * C_ds - r_mfs * C_mfs;
gamma = 1 / (C_ds * C_fr - C_mfs^2);

%% System dynamics and output 
f = [u(4)*x(5) - p/2*x(2)*x(4) - x(1)/tau_1;
    u(5)*x(5) + p/2*x(1)*x(4) - gamma*tau_3*x(2) - gamma*tau_3*x(3);
    u(2) - gamma*tau_4*x(2) + gamma*tau_5*x(3);
    -beta*x(4)/J + u(1)/J + 3*p*C_fr*x(2)^2/(2*J) + 3*p*C_mfs*x(2)*x(3)/(2*J) - 3*p*gamma*x(1)*x(2)/(2*J) - 3*p*gamma*C_ds*x(1)*x(3)/(2*J);
    N*u(3)*u(6)/L_dc - r_dc * x(5)/L_dc - 3*gamma*C_fr*u(4)*x(2)/(2*L_dc) - 3*gamma*C_mfs*u(4)*x(3)/(2*L_dc) - 3*gamma*C_mfs*u(5)*x(2)/(2*L_dc) - 3*gamma*C_ds*u(5)*x(3)/(2*L_dc)
    ];

y = [v_qs; v_ds; v_fr; w; i_dc];

%% Check if the solution solves the ODE
xsol = ...
    [subs(C_qs, 512e-9)*subs(v_qs, 100e3);
    subs(C_ds, 512e-9)*subs(v_ds,0) - subs(C_mfs, 82e-9)*subs(175e3);
    -subs(C_mfs, 82e-9)*subs(v_ds,0) + subs(C_fr, 210e-9)*subs(v_fr, 175e3);
    subs(w, 72);
    (subs(m_fe, 0.5) * subs(N,10) * subs(v_in, 15e3) - 1.5 * subs(m_q, 0.5) * subs(v_qs, 100e3)) / subs(r_dc, 12)];
usol = ...
    [subs(T_l, 0.67e6);
    subs(i_fr, subs(v_fr, 175e3)/subs(r_fr, 15e6));
    subs(m_fe, 0.5);
    subs(m_q, 0.5);
    subs(m_d, 0.5);
    subs(v_in, 15e3);
    ];

dxdt = subs(f, [x;u], [xsol;usol]);
if all(dxdt==jacobian(xsol,t))
    disp('xsol solves the ODE for usol');
else
    disp('xsol does NOT solve the ODE for usol');
end

%% Compute the linearization
As = jacobian(f, x);
Bs = jacobian(f, u);
Cs = jacobian(y, x);
Ds = jacobian(y, u);

A = subs(As, [x;u], [xsol;usol]);
B = subs(Bs, [x;u], [xsol;usol]);
C = subs(Cs, [x;u], [xsol;usol]);
D = subs(Ds, [x;u], [xsol;usol]);
