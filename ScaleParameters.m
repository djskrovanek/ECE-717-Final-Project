function paramMap = ScaleParameters()

% scale pu parameters from lab SEM to a new power level, keeping pu values

% machine 1 is the lab machine
% machine 2 is the theoretical scaled machine

% assumes both machines are non-salient (round rotor)

% Note W_m in this script denotes mechanical speed, whereas W denotes electrical
% This is different notation from proposal

%% Input parameters

% Machine 1 (Lab SEM) parameters
P_R1 = 470; % [W]
W_mR1 = 500*2*pi/60; % [rad/s]
V_R1 = 4e3; % peak line-neutral stator voltage
Vf_R1 = 7e3; % rated field excitation voltage
eta_1 = 0.85; % efficiency at rated op (unclear if this includes drag or just electrical)

R_s1 = 1.6e6; % Peter wrote R_s = 1.6 Meg, Baoyun wrote 1.7 Meg
C_s1 = 13.8e-9;
C_m1 = 2.2e-9; % Peter wrote C_m = 2.2e-9, Baoyun wrote 2.03e-9
p1 = 96; % poles
R_fr1 = 52.6e6;
C_lfr1 = 3.41e-9;

% Peter's drive parameters
L_dc1 = 3.4;
R_dc1 = 40;
NVin1 = 7.4*280; % N*Vin


% desired ratings for our machine (machine 2)
P_R2 = 1e6;
V_R2 = 100e3; % line-neutral stator voltage
p2 = 96; % pole count
W_mR2 = 1.5; % rad/s

H2 = 3.5; % inertia constant [s]
beta2 = 10; % [Nm/s], this value was assumed. Scaling lab SEM beta would give too much friction.

V_in2 = 10e3; % dc input (grid) voltage [V] 

%% calculations


% calculate base values for machine 1

P_B1 = P_R1;
V_B1 = V_R1;
W_mB1 = W_mR1;
W_B1 = p1*W_mB1; % note for electrostatic machine w_r=p*w_rm instead of p/2*w_rm

T_R1 = 1; % rated torque [pu]
I_B1 = P_B1/(3*V_B1); % [A]
Z_B1 = V_B1/I_B1; % [Ohm]

% calculate pu parameters

R_s1_pu = R_s1/Z_B1;
X_cs1 = 1/(W_B1*C_s1);
X_cs1_pu = X_cs1/Z_B1;
X_cm1 = 1/(W_B1*C_m1);
X_cm1_pu = X_cm1/Z_B1;
R_fr1_pu = R_fr1/Z_B1;
X_lfr1_pu = 1/(W_B1*C_lfr1)*1/Z_B1; 

X_Ldc1_pu = W_B1*L_dc1/Z_B1;
R_dc1_pu = R_dc1/Z_B1;
NVin1_pu = NVin1/V_B1;


% calculate base quantities for machine 2
P_B2 = P_R2;
V_B2 = V_R2;
W_mB2 = W_mR2;
W_B2 = p2*W_mB2; % note for electrostatic machine w_r=p*w_rm instead of p/2*w_rm

T_B2 = P_B2/W_mB2;
I_B2 = P_B2/(3*V_B2); % [A]
Z_B2 = V_B2/I_B2; % [Ohm]

% scale drive parameters

% keep same pu reactance X_L and resistance
L_dc2 = Z_B2*X_Ldc1_pu/W_B2;
R_dc2 = Z_B2*R_dc1_pu;

NVin2 = NVin1_pu*V_B2;
N2 = NVin2/V_in2;

% scale machine reactances, keeping same pu


X_cs2 = X_cs1_pu*Z_B2; % ohm
C_s2 = 1/(W_B2*X_cs2); % F

X_cm2 = X_cm1_pu*Z_B2;
C_m2 = 1/(W_B2*X_cm2); % F
C_mfs2 = C_m2; % C_m in simplified model is C_mfs in dq model

X_lfr2 = X_lfr1_pu*Z_B2; 
C_lfr2 = 1/(W_B2*X_lfr2);

C_fr2 = C_mfs2 + C_lfr2;

% assuming no saliency
C_ds2 = C_s2;
C_qs2 = C_s2;

C_lds2 = C_ds2 - C_mfs2;

% scale machine resistances, keeping same pu
R_s2 = R_s1_pu*Z_B2;
R_fr2 = R_fr1_pu*Z_B2;

% assuming round rotor
R_ds2 = R_s2;
R_qs2 = R_s2;

% assuming the same "leakage ratio" between field leakage conductance and total
% field conductance exists as between the field leakage capacitance and total field capacitance, 
% since C = epsilon*A/d and conductance G = 1/rho*A/d so they should be proportional

leakageRatio = C_lfr2/C_fr2;
G_fr2 = 1/R_fr2; % conductance [Siemens]
G_lfr2 = leakageRatio*G_fr2;
G_mfs2 = G_fr2-G_lfr2;
R_lfr2 = 1/G_lfr2;
R_mfs2 = 1/G_mfs2;

G_ds2 = 1/R_ds2;
G_lds2 = G_ds2 - G_mfs2;
R_lds2 = 1/G_lds2;

J2 = 2*P_R2*H2/W_mR2^2; % [kg*m^2]

% save all params to a file
paramNames = {'C_lds', 'C_lfr', 'C_mfs', 'C_qs', 'C_ds', 'C_fr', 'z', ...
    'L_dc', 'r_dc', 'J', 'beta', 'r_lds', 'r_lfr', 'r_mfs', 'r_qs', ...
    'r_ds', 'r_fr', 'v_in', 'N'};
paramVals = {C_lds2, C_lfr2, C_mfs2, C_qs2, C_ds2, C_fr2, p2,...
    L_dc2, R_dc2, J2, beta2, R_lds2, R_lfr2, R_mfs2, R_qs2,...
    R_ds2, R_fr2, V_in2, N2};
%paramTable = table(paramNames, paramVals);
paramMap = containers.Map(paramNames, paramVals);
end




