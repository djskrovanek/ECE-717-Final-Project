clear all; close all; clc;

% numerically calculates an equilibrium point for system given a set of 
% parameters and operating point choices

% operating point (some set manually, rest are solved for)
V_ds = 0;
V_qs = 100e3;
V_fr = 175e3;
We = 72; % electrical speed
W = 1.5; % mechanical speed (rad/s)
I_dc = 8; % Somewhat arbitrarily picked I_dc
V_in = 15e3; %5e3; % may want to consider changing V_in

% parameters
C_lds = 430e-9;
C_lfr = 128e-9;
C_mfs = 82e-9;
C_qs = 512e-9;
p = 96;
L_dc = 12;
r_dc = 12;
J = 3.1e6;
beta = 10;
N = 10;
r_lds = 82e3;
r_lfr = 15e6;
r_mfs = 420e3;
r_qs = 502e3;



% derived parameters
C_ds = C_lds + C_mfs;
C_fr = C_lfr + C_mfs;
r_ds = r_mfs + r_lds;
r_fr = r_lfr + r_mfs;

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
Q_qs = C_qs*V_qs
Q_ds = C_ds*V_ds-C_mfs*V_fr
Q_fr = -C_mfs*V_ds+C_fr*V_fr

% solve for required I_fr from Q_fr dot = 0
I_fr = Q_ds*gamma*alpha_3+Q_fr*gamma*alpha_4

% solve for required tau_l from omega dot = 0
Tau_l = beta*W-3*p/2*Q_ds*V_qs+3*p/2*Q_qs*V_ds

% solve for required value of I_qs=M_q*I_dc and I_ds=M_d*I_dc
I_qs = p/2*W*Q_ds+Q_qs/tau_1 % I_qs = M_q*I_dc
I_ds = -p/2*W*Q_qs+Q_ds*gamma*alpha_1+Q_fr*gamma*alpha_2 % I_ds = M_d*I_dc

% pick I_dc because that leads to linear equations to solve for M_q, M_d,
% and M_fe
M_q = I_qs/I_dc
M_d = I_ds/I_dc

% solve for required M_fe based on I_dc dot = 0
M_fe= 1/(N*V_in)*(I_dc*r_dc+3/2*M_q*V_qs+3/2*M_d*V_ds)

% solve for mech power
P_m = -Tau_l*W
% solve for output power to grid
P_out = M_fe*N*V_in*I_dc

% calc efficiency
Eta = P_out/P_m





