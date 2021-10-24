function [eqPts] = CalcEquilibrium()

% numerically calculates an equilibrium point for system given a set of 
% parameters and operating point choices

% operating point (some set manually, rest are solved for)
v_ds = 0;
v_qs = 100e3;
v_fr = 175e3;
we = 72; % electrical speed
w = 1.5; % mechanical speed (rad/s)
i_dc = 24; % 8 % Somewhat arbitrarily picked I_dc
v_in = 15e3; %5e3; % may want to consider changing V_in

% parameters

N = 10; % N is picked somewhat arbitrarily

% read rest of parameters from table
paramMap = ScaleParameters();

C_lds = paramMap('C_lds');
C_lfr = paramMap('C_lfr');
C_mfs = paramMap('C_mfs');
C_qs = paramMap('C_qs');
z = paramMap('z');
L_dc = paramMap('L_dc');
r_dc = paramMap('r_dc');
J = paramMap('J');
beta = paramMap('beta');
r_lds = paramMap('r_lds');
r_lfr = paramMap('r_lfr');
r_mfs = paramMap('r_mfs');
r_qs = paramMap('r_qs');
C_ds = paramMap('C_ds');
C_fr = paramMap('C_fr');
r_ds = paramMap('r_ds');
r_fr = paramMap('r_fr');


%{
% original parameters
C_lds = 430e-9;
C_lfr = 128e-9;
C_mfs = 82e-9;
C_qs = 512e-9;
z = 96;
L_dc = 12;
r_dc = 12;
J = 3.1e6;
beta = 10;
r_lds = 82e3;
r_lfr = 15e6;
r_mfs = 420e3;
r_qs = 502e3;

% derived parameters
C_ds = C_lds + C_mfs;
C_fr = C_lfr + C_mfs;
r_ds = r_mfs + r_lds; % note this is incorrect
r_fr = r_lfr + r_mfs; % note this is incorrect
%}


% derived parameters

tau_1 = r_qs * C_qs;
%tau_2 = r_ds * C_fr - r_mfs * C_mfs;
%tau_3 = r_ds * C_mfs - r_mfs * C_ds;
%tau_4 = r_fr * C_mfs - r_mfs * C_fr;
%tau_5 = r_fr * C_ds - r_mfs * C_mfs;
alpha_1 = C_fr/r_ds - C_mfs/r_mfs;
alpha_2 = C_mfs/r_ds - C_ds/r_mfs;
alpha_3 = C_mfs/r_fr - C_fr/r_mfs;
alpha_4 = C_ds/r_fr - C_mfs/r_mfs;
gamma = 1 / (C_ds * C_fr - C_mfs^2);



% calculate charges
Q_qs = C_qs*v_qs;
Q_ds = C_ds*v_ds-C_mfs*v_fr;
Q_fr = -C_mfs*v_ds+C_fr*v_fr;

% solve for required I_fr from Q_fr dot = 0
I_fr = Q_ds*gamma*alpha_3+Q_fr*gamma*alpha_4;

% solve for required tau_l from omega dot = 0
tor_l = beta*w-3*z/2*Q_ds*v_qs+3*z/2*Q_qs*v_ds;

% solve for required value of I_qs=M_q*I_dc and I_ds=M_d*I_dc
i_qs = z/2*w*Q_ds+Q_qs/tau_1; % I_qs = M_q*I_dc
i_ds = -z/2*w*Q_qs+Q_ds*gamma*alpha_1+Q_fr*gamma*alpha_2; % I_ds = M_d*I_dc

% pick I_dc because that leads to linear equations to solve for M_q, M_d,
% and M_fe
m_q = i_qs/i_dc;
m_d = i_ds/i_dc;

% solve for required M_fe based on I_dc dot = 0
m_fe= 1/(N*v_in)*(i_dc*r_dc+3/2*m_q*v_qs+3/2*m_d*v_ds);

% solve for mech power
P_m = -tor_l*w;
% solve for output power to grid
P = m_fe*N*v_in*i_dc;

% calc efficiency
Eta = P/P_m;

vars = {'v_ds'; 'v_qs'; 'v_fr'; 'we'; 'w'; 'i_dc'; 'v_in'; 'C_lds'; 'C_lfr'; ...
    'C_mfs'; 'C_qs'; 'z'; 'L_dc'; 'r_dc'; 'J'; 'beta'; 'N'; 'r_lds';...
    'r_lfr'; 'r_mfs'; 'r_qs'; 'C_ds'; 'C_fr'; 'r_ds'; 'r_fr'; 'tau_1'; 'alpha_1';...
    'alpha_2'; 'alpha_3'; 'alpha_4'; 'gamma'; 'Q_qs'; 'Q_ds'; 'Q_fr'; 'i_fr';...
    'tor_l'; 'i_qs'; 'i_ds'; 'm_q'; 'm_d'; 'm_fe'; 'P_m'; 'P'; 'Eta'};
vals = [v_ds; v_qs; v_fr; we; w; i_dc; v_in; C_lds; C_lfr; ...
    C_mfs; C_qs; z; L_dc; r_dc; J; beta; N; r_lds;...
    r_lfr; r_mfs; r_qs; C_ds; C_fr; r_ds; r_fr; tau_1; alpha_1;...
    alpha_2; alpha_3; alpha_4; gamma; Q_qs; Q_ds; Q_fr; I_fr;...
    tor_l; i_qs; i_ds; m_q; m_d; m_fe; P_m; P; Eta];
eqPts = table(vars, vals);
end


