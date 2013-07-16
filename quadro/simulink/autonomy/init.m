%% Init file for QK-model.mdl
clear all;
clc;

parameter_QK;                       % load Quadrotor parameters
parameter_regler;                   % Controller parameters
dt = 0.01;                          % sample rate
simLengthSeconds = 100;             % simulation length in seconds

var_position = 0;
var_accel    = 0;
var_gyro     = 0;

%% Modifikations by Max Rödelsperger

%Abmessungen QK
QKB     = (l_m+R)*2;                                    % Breite und Laenge ueber Alles 
LGH     = 0.15;                                         % hoehe Landegestell rel. zum Schwerpunkt
LGB     = 0.3;                                          % breite Landegestell rel. zum Schwerpunkt
LGL     = LGB;                                          % laenge Landegestell rel. zum Schwerpunkt
bGestell= 0.3;                                          % Dämpfung Gestell
kGestell= 100;                                          % Federsteifigkeit gestell

% verhaken
hgras = .1;                                             % höhe der grasnarbe
bgras = 0.5;                                            % dämpfung durch die grasnarbe
kgras = kGestell*2/3;                                   % Federsteifigkeit gestell und gras bei verhaken
verh  = [0 0 0 0];                                      % verhaken an gestelleckpunkt 0..3 aktiviert
verht = 0;                                              % ... zum zeitpunkt.

% Startposition
X_start= 0;                                             % Startposition
Y_start= 0;                                             % "
Z_start= -.155;                                         % Start von Bodenhoehe aus
%phi_start=-atan(neigung/100);                           % Startneigung entspricht Bodenneigung
%theta_start=0;                                          
%psi_start=0;

% Filter
t_filter= 0.005;

parameter_ZA;                       %lade Parameter des Zustandsautomaten