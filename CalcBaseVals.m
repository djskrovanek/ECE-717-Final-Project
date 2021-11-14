% returns an array of base values, along with vectors which have
% bases for u, x, and y.
% note: call x./x_B to normalize each component of x to pu
% requires x have dimensions 6 x k, where k is number of time points
function [baseVals, u_B, x_B, y_B] = CalcBaseVals

    % Reference: parameters from calling CalcEquilibrium
    %{
    vars = {'V_ds'; 'V_qs'; 'V_fr'; 'We'; 'W'; 'I_dc'; 'V_in'; 'C_lds'; 'C_lfr'; ...
    'C_mfs'; 'C_qs'; 'z'; 'L_dc'; 'r_dc'; 'J'; 'beta'; 'N'; 'r_lds';...
    'r_lfr'; 'r_mfs'; 'r_qs'; 'C_ds'; 'C_fr'; 'r_ds'; 'r_fr'; 'tau_1'; 'alpha_1';...
    'alpha_2'; 'alpha_3'; 'alpha_4'; 'gamma'; 'Q_qs'; 'Q_ds'; 'Q_fr'; 'I_fr';...
    'Tor_l'; 'I_qs'; 'I_ds'; 'M_q'; 'M_d'; 'M_fe'; 'P_m'; 'P'; 'Eta'};
    %}

    params = CalcEquilibrium();

    % assign local variables
    P_m = double(table2array(params(42,2)));
    W = double(table2array(params(5,2)));
    z = double(table2array(params(12,2)));
    V_qs = double(table2array(params(2,2)));
    V_in = double(table2array(params(7,2)));
    N = double(table2array(params(17,2)));
    
    % note: per unit system is split into 3 zones to normalize values
    % reasonably based on the nominal voltages/currents in that zone.
    
    % Base values common to all zones
    
    P_B = -P_m; % base power (flip sign so base power is positive)
    W_B = W; % base mechanical speed
    We_B = z*W_B; % base electrical frequency [rad/s]
    T_B = P_B/W_B; % base torque
    
    % Zone 1: machine/ac side
    V_B = V_qs; % base voltage for ac side
    % Note: P=3/2*V*I because we are using peak, not rms
    I_B = P_B/(3/2*V_B); % base current for ac side
    Q_B = I_B/We_B; % base charge [Coloumb]
    
    % Zone 2: DC input
    % Note: (only affects Vin, which has a transformer with turns ratio N in
    % the dc-dc converter between it and the rest of the DC side)
    Vin_B = V_in; % base voltage for dc input side
    
    % Zone 3: DC link
    Vdc_B = V_in*N; % base voltage for dc circuit
    Idc_B = P_B/Vdc_B; % base current for dc circuit

    % vectors to normalize input/state/output to PU
    % note: call x./x_B to normalize each component of x to pu
    % requires x have dimensions 6 x k, where k is number of time points
    x_B = [Q_B; Q_B; Q_B; W_B; Idc_B]; % base values for each state
    y_B = [V_B; V_B; V_B; W_B; Idc_B; P_B]; % base values for each state
    u_B = [T_B; I_B; 1; 1; 1; Vin_B]; % base values for each state

    % return an array
    baseVals = [P_B; W_B; We_B; T_B; V_B; I_B; Q_B; Vin_B; ...
    Vdc_B; Idc_B];

    % build a table of base variables to their value
    %{
    vars = {'P_B'; 'W_B'; 'We_B'; 'T_B'; 'V_B'; 'I_B'; 'Q_B'; 'Vin_B'; ...
    'Vdc_B'; 'Idc_B'; 'x_B'; 'y_B'; 'u_B'};
    vals = [P_B; W_B; We_B; T_B; V_B; I_B; Q_B; Vin_B; ...
    Vdc_B; Idc_B; x_B; y_B; u_B];
    baseVals = table(vars, vals);
    %}
end