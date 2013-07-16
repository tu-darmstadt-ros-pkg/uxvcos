%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Initialization of global variables
% NOTES: 
%  * everything in SI unit
%  *
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath '../tools' % the wind data is in the tools
addpath '..'        % ports.lib

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
command_on = 1;

% Motor Model for HIL (simple gain)
Fz_gain =   1 + 0.05*rand(1);

%% Inertia

% Iq = [0.0605 0 0; 0 0.0605 0; 0 0 0.018]; % inertia of the quadrotor,
% based on identification Xie --> FULL OF ERRORS
% Iq = [0.009 0 0; 0 0.009 0; 0 0 0.018]; % S.H., rechnerische Werte (wohl fehlerhaft), um die z-Achse muss wegen der 4 Motoren ca. 2x so hoch sein wie um die anderen Achsen

% Final inertia based on identification
% HIL_DI\Data\Identification_Ixx_yy  and HIL_DI\Data\Identification_Izz

% I_xx = 0.013;
% I_yy = 0.01;

% mean:
% I_xx = 0.0115;
% I_yy = 0.0115;

% I_xx = 0.0324;
% I_yy = 0.0324;

% I_xx = 0.0144;  % Identification to correct for F_mot_Force
% I_yy = 0.0144;

I_xx = 0.015;    % Working 
I_yy = 0.015;

I_zz = 0.0281;

Iq = [I_xx 0 0 ; 0 I_yy 0 ; 0 0 I_zz ];

% Berechnung: I_xx = 0.08 * .25^2 * 2

% resistance and winddata
rho_air = 1.2041; % air, 20 degrees celsius


wind_on = 0;
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

l = 0.25;             %[m]        Abstand Motor zu Kern

% simplified motor model
% T_Mot_Force     = 0.0469;     % not needed

K_Mot_Force     = 0.3512 ;  

% K_Mot_Torque    = 0.0013;     % old
K_Mot_Torque    = 0.013;     % S.H. 27.03.2013 --> lt. Disse Alex --> all identification based on this value
% TD_Mot_Torque   = 1.9541;     % not needed
% not needed:
% k_m     = 0.0087;  %[Nm/A]     Motorkonstante (aus Datenblatt 2824-34)


%% Rotational Disturbances
Tm = 10;
deltaTm = 0.2;
Mm = 0.0;
% Mm = 0.5;


% Low pass for RefAttitude Sensor
T_refalt = 0.01;    % not needed

% Calibration Reference Angles
% Obtained from measurement with QR fixed
Bias_Phi    = -0.0088;
Bias_Theta  = 0.022;
% Calibration Yaw-Angle not needed since only Ref-Angle is used here.

%% Noise
Noise_on = 0;
seedvector = randseed(1,7);

std_dev_x           = 1e-1 * Noise_on;
% std_dev_x           = 1e-2 * Noise_on;  % reduced due to bad PID-Performance
% std_dev_x           = 0;  % no noise for PID-Testing and comparisson
sample_time_noise_x = 0.2;
std_dev_y           = 1e-1 * Noise_on;
% std_dev_y           = 1e-2 * Noise_on;  % reduced due to bad PID-Performance
% std_dev_y           = 0;  % no noise for PID-Testing and comparisson
sample_time_noise_y = 0.2;
std_dev_z           = 0.03 * Noise_on;
sample_time_noise_z = 0.1;

seed_x = seedvector(1);
seed_y = seedvector(2);
seed_z = seedvector(3);

std_dev_xp              = 0.03 * Noise_on;
sample_time_noise_xp    = 0.01;
std_dev_yp              = 0.03 * Noise_on;
sample_time_noise_yp    = 0.01;
std_dev_zp              = 0.03 * Noise_on;
sample_time_noise_zp    = 0.01;

seed_xp = seedvector(4);
seed_yp = seedvector(5);
seed_zp = seedvector(6);

% 26.02.2013, S.H.
% Noise in psi added since no KF is used any more
std_dev_psi             =  1 * pi/180 * Noise_on;
seed_psi                =  seedvector(7);
sample_time_noise_psi   = 0.1;


%% Reference System
% coeffiecients of  denominator for the REFERENCE system  (also used to smooth the command of the cascade controller.

% T_x_ref = 0.8113;   % for x_p_max = 3, x_pp_max = 1, MC: High Uncertainty
% T_y_ref = 0.8113;   % 

% T_x_ref = 0.9425;   % for x_p_max = 3, Alpha_max = 7 deg, MC: High Uncertainty
% T_y_ref = 0.9425;   % 

T_x_ref = 0.555;   % for x_p_max = 1.5, Alpha_max = 7 deg, MC: High Uncertainty
T_y_ref = 0.555;   % 

% T_x_ref = 1.24;   % for x_p_max = 5, Alpha_max = 7 deg, MC: High Uncertainty
% T_y_ref = 1.24;   % 

% T_x_ref = 1.2363;   % for x_p_max = 5, xpp_max = 1m/s , MC: High Uncertainty
% T_y_ref = 1.2363;   % 

T_z_ref = 1;
T_psi_ref = 1;

% denominator coefficient for ptn(n=relative degree)
den_x_ref = poly( ones(1,4) * -1 * T_x_ref);
den_y_ref = poly( ones(1,4) * -1 * T_y_ref);
den_z_ref = poly( ones(1,4) * -1 * T_z_ref);
den_psi_ref = poly( ones(1,2) * -1 * T_psi_ref);

% rate limitation for the inputs

x_p_lim = 1.5;    % Given by user, for x_p_max = 3, x_pp_max = 1
y_p_lim = 1.5;    % Given by user, for x_p_max = 3

z_p_lim = 1;
psi_p_lim = 10*pi/180;

% damping constant for the dynamic system
dtran = 0.01;   % linear approximation, see inversion_new.m

% No uncertainty needed, rotational motion is physically present
drot = 0.0117;  % Based on final I_yy estimation

% satuaration limit of the model
Fz_up = 0;
Fz_lo = - 1.2 * m * g;
% Fz_lo = - 1.5 * m * g;

Mx_up = 0.20;
% Mx_up = 0.60;   % identification
% Mx_up = 0.1;   % show saturation (too low)
Mx_lo = - Mx_up;

My_up = 0.20;
% My_up = 0.60;   % identification
% My_up = 0.1;   % show saturation (too low)
My_lo = - My_up;

Mz_up = 0.002;
Mz_up = 0.02;   % S.H. 27.03. --> according to K_Mot_Torque
Mz_lo = - Mz_up;

%%
% Error Dynamic with integrator
s       = tf('s');
Fs_5      = 1/s^5;  
Fs_3      = 1/s^3;  

Pole_x  = -1.5*[1.001 1.002 1.003 1.004 1.005];   % Pole in x-Richtung
%Pole_x  = -1.5*[0.5 1.001 1.002 2.1 2];   % Pole in x-Richtung

ZRD     = ss(Fs_5);   % ZRD benötigt für place.m

K_x     = place(ZRD.a, ZRD.b, Pole_x);      % Regler auslegen
K_x     = fliplr(K_x);                      % Regler in gewohnte Form bringen

% Pole_y  = -1.*[1.001 1.002 1.003 1.004 1.005];   % Pole in x-Richtung
% Pole_y  = -1.5.*[1.001 1.002 1.003 1.004 1.005];   % Pole in x-Richtung
Pole_y = Pole_x;

K_y     = place(ZRD.a, ZRD.b, Pole_y);      % Regler auslegen
K_y     = fliplr(K_y);                      % Regler in gewohnte Form bringen

Pole_z  = -3 *[1.001 1.002 1.003 1.004 1.005];   % Pole in x-Richtung
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



%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Initialization for the simple cascaded control
% NOTES: 
%  * everything in SI unit
%  *
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Not used in Simulation:
% PWM2N  = 0.0325; 
% v2N = PWM2N * 255 /13;
% lm = 0.25; % length of the lever arm


% postion controller (PD)
% k_p_position = 1; % Xie
% k_d_position = 1;

% S.H. 04.04.2013 --> adaption needed due to artificial sensor noise
k_p_position    = 1;
k_d_position    = 1;


% v pitch controller (PI)
k_p_v_pitch = 0.5*0.1;
k_i_v_pitch = 0.5*0.01;

% theta controller (PID)
% k_p_theta = 1.4*0.5; % Xie
% k_i_theta = 1.4*0.25;
% k_d_theta = 1.4*0.25;
k_p_theta = 0.325;    % S.H. 04.04.2013 --> based on final param. Estimation
k_i_theta = 0.2625;
k_d_theta = 0.1;

% v_roll controller (PI)
k_p_v_roll = k_p_v_pitch ;
k_i_v_roll = k_i_v_pitch ;

% phi controller (PID)
k_p_phi = k_p_theta ;
k_i_phi = k_i_theta ;
k_d_phi = k_d_theta ;

% z controller (PID)
k_z_gain = 0.5;     % S.H. 27.03.2013 --> Versuch weniger Verstaerkung
k_p_z = 4 *k_z_gain;
k_i_z = 2 *k_z_gain;
k_d_z = 2 *k_z_gain;

% psi controller (PID)
% k_p_psi = 0.4*0.1;    % Xie
% k_i_psi = 0.4*0.05;
% k_d_psi = 0.4*0.05;

k_p_psi = 0.12;  % S.H. 02.04.2013 --> Based on final Estimation!
k_i_psi = 0.045;
k_d_psi = 0.08;


%% Load Reference Trajectories
cd ReferenceTrajectories;
ReferenceTrajectories;
cd ..
% cd ReferenceTrajectories\;
% ReferenceTrajectories;
% cd ..