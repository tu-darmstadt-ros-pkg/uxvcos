clear all
close all

parameter_QK;                       % load Quadrotor parameters
reglerparameter_QK;                 % Controller parameters

% adapted parameters, best fit with real data
k_t   = 0.015336864714397;
CT0s  = 1.538190483976698e-005;
C_wxy = 0.12;
C_wz  = 0.1;
C_mxy = 0.074156208;
C_mz  = 0.050643264;

% change rounding accuracy of sample time
Re = 6378137;
dt = 0.01;

%% init the rest

var_accel    = ([0.5677 0.6534 0.3599]).^2;     % Accelerometer                 (m/s^2)^2
var_gyro     = ([0.3458 0.2468 0.0246]).^2;     % Gyroscopes                    (rad/s)^2
var_height   = (0.838).^2;                      % Height sensor                 (m)^2
var_position = ([3 3 sqrt(var_height)]).^2;     % GPS and height sensor         (m)^2
var_velocity = (5e-1*ones(1,3)).^2;             % velocity sensor (GPS)         (m/s)^2
var_omega    = (10*ones(1,4)).^2;               % Motor revoultion speed sensor (rad/s)^2
var_magnetic = (4e-2*ones(1,3)).^2;             % Magnetometer                  (mT)^2

bias_accel   = [1, 0.4552, 0.0963]';      % max +-0.5m/s^2 bias   [-0.2 0.3 0.04]';
drift_accel  = 1e-4*ones(3,1);                  % [2e-5 -1e-5 0.5e-5]';
bias_gyro    = [-0.0033,-0.0010,-0.0058]';      % max +-0.1/s bias      [0.1 -0.03 0.05]';
drift_gyro   = 1e-4*ones(3,1);                  %[5e-5 -4e-5 6e-5]';

% B-Field when looking to the north in Darmstadt
B0           = [0.414730546772447 0.011142870512951 0.910146422495241]';

% Kalman parameters

x0 = [zeros(2,1);
      0;
      zeros(3,1);
      [1 0 0 0]';
    bias_accel;                               % Bias accel
    bias_gyro;                                % Bias gyro
    zeros(3,1);                                 % w_bnb
    zeros(4,1);                                 % Motorspeed
    k_t;                                        % k_t
    CT0s;                                       % CT0s
    0*[0.0911, 0.0551, -0.0079]';                 % Bias M    
    0*[0 4]';                                     % wind speed   
    C_wxy];                                     % drag coefficient          

% P-Matrix
P0              = zeros(length(x0));
P0(1:3,1:3)     = 0*diag([4 4 1e-2]);           % position
P0(4:6,4:6)     = 0*diag([4 4 7e-2]);           % velocity
P0(7:10,7:10)   = 0e-8*eye(4);                  % q
P0(11:13,11:13) = (0.5^2*eye(3));             % max 0.5m/s^2 accel_bias
P0(14:16,14:16) = 0.1*(0.1^2*eye(3));	        % max 0.1/s gyro_bias
P0(17:19,17:19) = 0*0.1*eye(3)*(sum(abs(bias_gyro)).^2);           % gyro
P0(20:23,20:23) = 1e+1*eye(4);                  % omega motor
P0(24,24)       = 0*1e-3*(k_t)^2;                 % k_t
P0(25,25)       = 0*1e-3*(CT0s)^2;                % CT0s
P0(26:28,26:28) = 0e-5*diag([1 1 1]);           % M_bias, x,y,z
P0(29:30,29:30) = 0*1*diag([1 1]);                % v_wind x,y
P0(31,31)       = 0*1*1e-4^2;                     % C_wxy

% Q-Matrix
Q0              = zeros(size(x0(1:end-1)));
Q0(1:3)         = 0e-13*ones(3,1);              % position
Q0(4:6)         = 1e-0*var_accel';              % velocity
Q0(7:9)         = 0e-13*ones(3,1);              % q, i.e. Euler angles
Q0(10:12)       = (1e-4*[1 1 1]).^2;            % accel_bias drift
Q0(13:15)       = (1e-4*[1 1 1]).^2;            % gyro_bias drift
Q0(16:18)       = 1e-0*var_gyro';               % gyro
Q0(19:22)       = 1*(14.8/2^8*ones(4,1)).^2;    % omega motor
Q0(23)          = 0*1*(1e-4*k_t)^2;               % k_t
Q0(24)          = 0*1*(1e-4*CT0s)^2;              % CT0s
Q0(25:27)       = 0e-5*[1 1 1];                 % M_bias
Q0(28:29)       = 0*1e-1*[1 1];                   % wind speed in nav-coordinates
Q0(30)          = 0*1e-9;                         % drag coefficients
Q0              = Q0*0.01;

R0              = [var_position(1:2) var_position(3) var_velocity var_accel var_gyro var_omega var_magnetic]';

Pos0            = [49.9 8.9]';
save parameter;

clear all
parameter = load('parameter');
