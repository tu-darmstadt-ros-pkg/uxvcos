%% Parameter für Regler
% Skalierungsfaktoren
PWM2N  = 0.0325; 
PWM2NM = 4.6350e-04; 

%% Höhenregler 

% PID-Regler
% Kr_h = 150*PWM2N;                          % Vorverstärkung
% Kp_h = 1;                             % P-Anteil
% Ki_h = 0.24;                             % I-Anteil
% Kd_h = 1;                             % D-Anteil

% Original 
Kr_h = 150*PWM2N;                          % Vorverstärkung
Kp_h = 0.6;                             % P-Anteil
Ki_h = 0.4;                             % I-Anteil
Kd_h = 1;                             % D-Anteil
%% Geschwindigkeitsregler

% PI-Regler(Geschwindigkeit)
Kr_w=1.5;                             % Vorverstärkung
Kp_w=0.03;                             % P-Anteil
Ki_w=0.005;                             % I-Anteil

% PID-Regler (Sollwinkel)
Kr_g= 8*PWM2N*2*l_m;                 % Vorverstärkung
Kp_g=2;                             % P-Anteil
Ki_g=1;                            % I-Anteil
Kd_g=1;                               % D-Anteil

% original
% PI-Regler(Geschwindigkeit)
% Kr_w=1.32;                                % Vorverstärkung
% Kp_w=0.8;                             % P-Anteil
% Ki_w=0.08;                            % I-Anteil
% 
% % PID-Regler (Sollwinkel)
% Kr_g= PWM2N*2*l_m;                     % Vorverstärkung
% Kp_g=60;                               % P-Anteil
% Ki_g=30;                               % I-Anteil
% Kd_g=60;                               % D-Anteil

%% Gierregler

% PID-Regler
Kr_y = 7/k_t*PWM2NM;                           % Vorverstärkung
Kp_y = 1;                              % P-Anteil
Ki_y = 0;                               % I-Anteil
Kd_y = 1;                               % D-Anteil