% init-file für das einfache Quadrokoptermodell
rho = 1.225;                                                % [kg/m³]   Dichte der Luft
g = 9.81;                                                   % [m/s²]    Erdbeschleunigung
w_e = 7.2921151467e-5;                                      % [rad/s]   Erddrehrate
c_w = 0.9228;                                               % [1]       Widerstandsbeiwert bei translatorischer Bewegung
c_M = 0.9228;                                               % [1]       Widerstandsbeiwert bei rotatorischer Bewegung
A_xy = 0.023*0.55 + .15^2;                                  % [m²]      Fläche in x-/y-Achse
A_z = 2*0.023*0.55 + .15^2 + 0.1269^2*pi*4;                 % [m²]      Fläche in z-Achse, mit Rotoren
l_m = 0.2775;                                               % [m]       Abstand Motor zu Kern
z_m = -0.02;                                                % [m]       z-Abstand Propeller-Schwerpunkt
m = 1.316;                                                  % [kg]      Gesamtmasse des Quadrokopters
m_M = 0.05;                                                 % [kg]      Masse Motor + Propeller 
m_R = 0.05;                                                 % [kg]      Masse Rohr
m_C = m - 4*m_M - 2*m_R;                                    % [kg]      Masse Kern
I_x = 2*m_M*l_m^2+1/12*m_C*(2*0.15^2)+1/12*m_R*(2*l_m)^2;   % [kg m²]   Massenträgheitsmoment um x-Achse
I_y = I_x;                                                  % [kg m²]   Massenträgheitsmoment um y-Achse
I_z = 4*m_M*l_m^2+1/12*m_C*(2*0.15^2)+2*1/12*m_R*(2*l_m)^2; % [kg m²]   Massenträgheitsmoment um z-Achse
inertia = diag([I_x I_y I_z]);                              % [kg m²]   Massenträgheitstensor des Quadrokopters
U_max = 14.8;                                               % [V]       Maximale Eingangsspannung für Motor

% Motordaten aus Roxxy2827-34 Identi mit 10x4.5 Schraube
Psi =  0.011099903912915;
J_M =  1.756700271991711e-005;
Mw1 = -7.549213251712553e-005;
Mw2 =  3.073649736006492e-007;
R_A =  0.506251525173286;

%Fw0 = -0.121225084369419;
Fw1 = -0.001823902972521;
Fw2 =  1.712938521037881e-005;

% Propellerdaten zur Schubberechnung
R = 0.1269;                                            % Rotorradius

% angepasste Version, von Hand
CT3 = -0.0350;
CT2 =  0.0081;
CT1 = -0.0100;
CT0 =  0.0133;

% CM = CM3*J^3 + CM2*J^2 + CM1*J + CM0
CM3 = -7.2696e-004;
CM2 = -6.6376e-005;
CM1 =  8.0233e-005;
CM0 =  1.8968e-004;
% Moment M = CM*rho*pi*R^4*omega^2

%
% Hilfsvariablen Luftwiderstand (Werte einsetzen)
C_wxy   = 1/2*A_xy*rho*c_w;                                 % bezogene Widerstandsgröße udot, vdot
C_wz    = 1/2*A_z*rho*c_w;                                  % bezogene Widerstandsgröße wdot
C_mxy   = 1/2*A_z*rho*c_M;                                  % bezogene Widerstandsgröße pdot, qdot
C_mz    = 1/2*A_xy*rho*c_M;                                 % bezogene Widerstandsgröße rdot

w0 =  -Fw1/(2*Fw2) + sqrt((Fw1/(2*Fw2))^2 + 0/4*m*g/Fw2);
U0 = R_A/Psi*((Psi^2/R_A + Mw1)*w0 + Mw2*w0^2);
F0 = m*g;

% Offset torques due to not symmetrical mass distribution
M_x = -7.1179e-5;%4.7125e-5;
M_y = 7.6312e-5;%1.0503e-4;
M_z = 1.4337e-3;%7.4911e-4;

%%
zaehler = (Fw1+2*Fw2*w0)*Psi/J_M/R_A;
nenner  =  (Psi^2-Mw1*R_A)/J_M/R_A + 2*Mw2*w0/J_M;
F_nach_u = tf(4*zaehler,[1 nenner]);
x_nach_u = F_nach_u*tf(1,[m 0 0]);