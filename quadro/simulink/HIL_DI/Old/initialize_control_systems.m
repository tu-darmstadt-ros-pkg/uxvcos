%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Initialization of global variables
% NOTES: 
%  * everything in SI unit
%  *
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all

addpath '../tools' % the wind data is in the tools
addpath '..' % Ports.mdl

% symbolic inversion of the system matrix
%inversion;

% sample time 
Ts = 1e-2; % sample frequency of the sensor = 100Hz

% const
g = 9.81;

% safety limit 
PWM_MAX = 200;

% inertia and mass
%m = 1.25; % real quadrotor
m = 0.30;
m_real = m * ( 1 + 0.05*rand(1));

% Whether input is non-zero
command_on = 0;

% Motor Model for HIL (simple gain)
Fz_gain =   1 + 0.05*rand(1);

%Iq = [0.009 0 0; 0 0.009 0; 0 0 0.018]; % inertia of the quadrotor
Iq = [0.0605 0 0; 0 0.0605 0; 0 0 0.018]; % inertia of the quadrotor
%Iq = [0.0505 0 0; 0 0.0505 0; 0 0 0.018]; % inertia of the quadrotor

% resistance and wind
rho_air = 1.2041; % air, 20 degrees celsius

wind_on = 1;
load_winddata; % in ./tools/

v_wind_max_x = 0; 
T_wind_x = 24;
phase_wind_x = 0;

v_wind_max_y = 0;
T_wind_y = 24;
phase_wind_y = 0;

v_wind_max_z = 2; 
T_wind_z = 24;
phase_wind_z = 0;

A_fx = 2e-2 * 0.4; % Area for Resistance 
cw_x =  1.05; % drag coeffient (cube)
A_fy = A_fx;
cw_y = cw_x;
A_fz = 2 * A_fx;
cw_z =  1.05; % drag coeffient (cube)


% MOTOR PARAMETERS
motor_model = 0; % 0 for simple gain, 1 for complex omega square model

% complex motor model 
J = 0; % advanced ratio	
%A_propellor = 0;
ct_rho_a4 = 0.2606; %  factor from regression


l = 0.2775;                                                   %[m]        Abstand Motor zu Kern
% simplified motor model
T_Mot_Force     = 0.0469;
K_Mot_Force     = 0.3512 ;
T_Mot_Torque    = 0.1886;
K_Mot_Torque    = 0.0013;
TD_Mot_Torque   = 1.9541;
k_m     = 0.0087;  %[Nm/A]     Motorkonstante (aus Datenblatt 2824-34)


% Rotational Disturbances
Tm = 4;
deltaTm = 0.1;
Mm = 0.01*0;


% Low pass for RefAttitude Sensor
T_refalt = 0.01;

% Noise
seedvector = randseed(1,6);

variance_noise_x = 1e-4;
variance_noise_y = 1e-4;
variance_noise_z = 1e-4;

variance_noise_x_KF = 1e-4;
variance_noise_y_KF = 1e-4;
variance_noise_z_KF = 1e-4;

seed_x = seedvector(1);
seed_y = seedvector(2);
seed_z = seedvector(3);

variance_noise_xp = 1e-4;
variance_noise_yp = 1e-4;
variance_noise_zp = 1e-4;

seed_xp = seedvector(4);
seed_yp = seedvector(5);
seed_zp = seedvector(6);

% Kalman Filter 
%variance_noise_phi = 1e-8;
%variance_noise_theta = 1e-8;
%variance_noise_psi = 1e-8;
%
%variance_noise_angularx = 1e-4;
%variance_noise_angulary = 1e-4;
%variance_noise_angularz = 1e-3;

P_0 = diag([ 1; 1; 1; 1; 1; 1;]) * 1e-4;
H_k = [eye(3) zeros(3,3)];
F_k = [eye(3) eye(3)*Ts; zeros(3,3) eye(3)];
R_k = diag([ variance_noise_x_KF variance_noise_y_KF variance_noise_z_KF ]);
Q_k_di = [ Ts^4/4*eye(3,3) Ts^3/2*eye(3,3); Ts^3/2*eye(3,3) Ts^2*eye(3,3)]*1e0;
%P_0 = diag([[ones(3,1) ; ones(3,1) ]*1e-4 ; [ones(3,1); ones(3,1)]*0.4] ) ;
%H_k = [ eye(3) zeros(3,9); ...
%        zeros(3,6) eye(3) zeros(3,3); ...
%        zeros(3,9) eye(3); ];
%%F_k = [eye(3) eye(3)*Ts; zeros(3,3) eye(3)]
%R_k = diag([ variance_noise_x_KF variance_noise_y_KF variance_noise_z_KF ...
%variance_noise_phi variance_noise_theta variance_noise_psi ...
%variance_noise_angularx variance_noise_angulary variance_noise_angularz ]);
%Qbase = [ Ts^4/4*eye(3,3) Ts^3/2*eye(3,3); Ts^3/2*eye(3,3) Ts^2*eye(3,3)];
%%Q_k = [ Qbase*Ts*30 Qbase*Ts/2; Qbase*Ts/2 Qbase;]  ;
%Q_k = [ Qbase*40 Qbase*Ts/2; Qbase*Ts/2 Qbase*5;] ;
%Q_k_trans = [ ones(6,6) zeros(6,6); zeros(6,12)] * 0e-8 ; 
%Q_k_rot_1 = [zeros(6,12) ; zeros(6,6) [ones(3,3) zeros(3,3);zeros(3,6)]; ] * 0e-1 ;
%Q_k_rot_2 = [zeros(6,12) ; zeros(6,6) [zeros(3,6);zeros(3,3) eye(3)]; ] * 0e-2 ;
%Q_k = Q_k + Q_k_trans + Q_k_rot_1 + Q_k_rot_2;

delta_Mx = 0.4;
delta_My = delta_Mx;
delta_Mz = delta_Mx;

errFx = 2;
errFy = errFx;
errFz = errFx; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Initialization for the simple dynamic extension model
% NOTES: 
%  *
%  *
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coeffiecients of  denominator for the REFERENCE system (relative degree 5 5 5 3
T_x_ref = 1.5;
T_y_ref = 1.5;
T_z_ref = 1.0;
T_psi_ref = 1;

% denominator coefficient for ptn(n=relative degree)
den_x_ref = poly( ones(1,4) * -1 / T_x_ref);
den_x_ref = den_x_ref / den_x_ref(end);
den_y_ref = poly( ones(1,4) * -1 / T_y_ref);
den_y_ref = den_y_ref / den_y_ref(end);
den_z_ref = poly( ones(1,4) * -1 / T_z_ref);
den_z_ref = den_z_ref / den_z_ref(end);
den_psi_ref = poly( ones(1,2) * -1 / T_psi_ref);
den_psi_ref = den_psi_ref / den_psi_ref(end);

% rate limitation for the inputs
x_p_lim = 1.2;
y_p_lim = 1.2;
z_p_lim = 1.4;
psi_p_lim = 5;


% damping constant for the dynamic system
dtran = 0.003; %FIXME: the value should be adapted in the real model
drot = 0.2;%FIXME: the value should be adapted in the real model

% satuaration limit of the model
Fz_up = 0;
Fz_lo = - 1.2 * m * g;

Mx_up = 0.6;
Mx_lo = - Mx_up;

My_up = 0.6;
My_lo = - My_up;

Mz_up = 0.002;
Mz_lo = - Mz_up;


% Error Dynamic with integrator
s       = tf('s');
Fs_5      = 1/s^5;  
Fs_3      = 1/s^3;  

Pole_x  = -1*[1.001 1.002 1.003 1.004 1.005];   % Pole in x-Richtung

ZRD     = ss(Fs_5);   % ZRD benötigt für place.m

K_x     = place(ZRD.a, ZRD.b, Pole_x);      % Regler auslegen
K_x     = fliplr(K_x);                      % Regler in gewohnte Form bringen

Pole_y  = -1*[1.001 1.002 1.003 1.004 1.005];   % Pole in x-Richtung

K_y     = place(ZRD.a, ZRD.b, Pole_y);      % Regler auslegen
K_y     = fliplr(K_y);                      % Regler in gewohnte Form bringen

Pole_z  = -[1.001 1.002 1.003 1.004 1.005];   % Pole in x-Richtung
K_z     = place(ZRD.a, ZRD.b, Pole_z);      % Regler auslegen
K_z     = fliplr(K_z);                      % Regler in gewohnte Form bringen

ZRD3     = ss(Fs_3);   % ZRD benötigt für place.m
Pole_psi  = -[1.001 1.002 1.003];   % Pole in x-Richtung
K_psi     = place(ZRD3.a, ZRD3.b, Pole_psi);      % Regler auslegen
K_psi     = fliplr(K_psi);                      % Regler in gewohnte Form bringen

K_cell = {K_x, K_y, K_z, K_psi};
y_dim = 4; % is the length of output y
old  = zeros(y_dim, 1); 
for ii=1:length(K_cell)
clear temp 
temp = zeros(y_dim, length(K_cell{ii}));
temp(ii,:) = K_cell{ii};
old =  [old temp];
end
K_err = old(:,2:end);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Initialization for the simple cascaded control
% NOTES: 
%  * everything in SI unit
%  *
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 
PWM2N  = 0.0325; 
v2N = PWM2N * 255 /13;
lm = 0.2775; % length of the lever arm


% postion controller (PD)
k_p_position = 2;
k_d_position = 1;

% v pitch controller (PI)
% FIXME: why not PD?
k_p_v_pitch = 0.1;
k_i_v_pitch = 0.01;

% theta controller (PID)
%k_p_theta = 120*PWM2N*lm;
%k_i_theta = 100*PWM2N*lm;
%k_d_theta = 60*PWM2N*lm;

k_p_theta = 0.5;
k_i_theta = 0.25;
k_d_theta = 0.25;

% v_roll controller (PI)
k_p_v_roll = k_p_v_pitch ;
k_i_v_roll = k_i_v_pitch ;

% phi controller (PID)
k_p_phi = k_p_theta ;
k_i_phi = k_i_theta ;
k_d_phi = k_d_theta ;

% z controller (PID)
k_p_z = 4;
k_i_z = 2;
k_d_z = 2;

% psi controller (PID)
k_p_psi = 0.4*0.1;
k_i_psi = 0.4*0.05;
k_d_psi = 0.4*0.05;


