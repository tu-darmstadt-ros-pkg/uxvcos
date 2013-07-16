%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% !!!DISSERTATIONS-FILE!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Script zum Erzeugen der Referenz-Trajektorien f�r die Evaluierung der
% Regler.
%
% Zwei Szenarien sind denkbar:
% 1) Indoor
% 2) Outdoor

% S. Haus, 18.04.2013


%%  Indoor-Szenario

Point_Start     =   [10      0  0   0   0];          % Start point
Point_01        =   [20     10 0   0   0];          % Gang entland fliegen
Point_02        =   [35     15 5 -4   0];           % Halle durchqueren und steigen
Point_03        =   [50     15 5 -4  30*pi/180];    % QR drehen f�r Foto
Point_04        =   [55     15 5 -4  -30*pi/180];   % QR drehen f�r Foto
Point_05        =   [60     15 5 -4  -30*pi/180];     % QR drehen in Ausgangslage
Point_06        =   [65     15 0 -4  -30*pi/180];     % Einfliegen in Schacht
Point_07        =   [70     15 0 15  45*pi/180];   % Sinkflug im Schacht und drehen f�r Fotos
Point_08        =   [90     13 0 15  45*pi/180];   % schnelle Manöver
Point_09        =   [95     14 1 15  45*pi/180];   % 
Point_10        =   [100    15 0 15  45*pi/180];   % 
Point_11        =   [105    14 1 14  45*pi/180];   % 


Mission_Indoor  = [Point_Start; Point_01; Point_02; Point_03; Point_04; Point_05; Point_06; Point_07; Point_08; Point_09; Point_10; Point_11];
% 
% % Plot indoor mission
% figure; 
% subplot(4,1,1);
% stairs(Mission_Indoor(:,1),Mission_Indoor(:,2));
% grid on; ylabel('x-Position');
% subplot(4,1,2);
% stairs(Mission_Indoor(:,1),Mission_Indoor(:,3));
% grid on; ylabel('y-Position');
% subplot(4,1,3);
% stairs(Mission_Indoor(:,1),Mission_Indoor(:,4));
% grid on; ylabel('z-Position');
% subplot(4,1,4);
% stairs(Mission_Indoor(:,1),Mission_Indoor(:,5)*180/pi);
% grid on; ylabel('Psi-Angle');

% Assign Mission_Indoor_steps for Simulink source block
Mission_Indoor_steps       = [];
Mission_Indoor_steps.time  = Mission_Indoor(:,1);
Mission_Indoor_steps.signals.values   = Mission_Indoor(:,2:5);
Mission_Indoor_steps.signals.dimensions          = 4;


%%  Outdoor-Szenario

Point_Start     =   [10      0  0   0   0];          % Start point
Point_01        =   [20     50 50 -20   0];          % Strecke fliegen und steigen
Point_02        =   [50     50 50 -40   -45*pi/180];           % Senkrecht steigen und gieren
Point_03        =   [80     50 50 -40   0];    % Zur�ck drehen
Point_04        =   [90     80 50 -40  0];   % In eine Richtung fliegen
Point_05        =   [120     0  0 -4  0];     % Zur�ck zum Startpunkt fliegen
Point_06        =   [160     0  0  0   0];     % Sinken in den Startpunkt

Mission_Outdoor = [Point_Start; Point_01; Point_02; Point_03; Point_04; Point_05; Point_06];

% % Plot indoor mission
% figure; 
% subplot(4,1,1);
% stairs(Mission_Outdoor(:,1),Mission_Outdoor(:,2));
% grid on; ylabel('x-Position');
% subplot(4,1,2);
% stairs(Mission_Outdoor(:,1),Mission_Outdoor(:,3));
% grid on; ylabel('y-Position');
% subplot(4,1,3);
% stairs(Mission_Outdoor(:,1),Mission_Outdoor(:,4));
% grid on; ylabel('z-Position');
% subplot(4,1,4);
% stairs(Mission_Outdoor(:,1),Mission_Outdoor(:,5)*180/pi);
% grid on; ylabel('Psi-Angle');

% Assign Mission_Outdoor_steps for Simulink source block
Mission_Outdoor_steps       = [];
Mission_Outdoor_steps.time  = Mission_Outdoor(:,1);
Mission_Outdoor_steps.signals.values   = Mission_Outdoor(:,2:5);
Mission_Outdoor_steps.signals.dimensions          = 4;

