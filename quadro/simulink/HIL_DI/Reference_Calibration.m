%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% !!!DISSERTATIONS-FILE!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Skript zum Kalibrieren der Reference-Sensoren des Pr�fstandes
%
% Pr�fstand muss fixiert sein in Nulllage, dann m�ssen die Referenz-Winkel
% ausgelesen werden.
%
% 
%
% S. Haus, 12.03.2013

clear Quatern

[Filename, Pathname]= uigetfile('*.csv','Choose file');

load([Pathname,Filename]);

eval(strcat('calibration = ', Filename(1:end-4), ';' ))

Quatern(:,1) = calibration(:,5);
Quatern(:,2) = calibration(:,2);
Quatern(:,3) = -calibration(:,3);
Quatern(:,4) = -calibration(:,4);

[Psi, Theta, Phi] = quat2angle(Quatern,'ZYX');

Bias_Phi    = mean(Phi)
Bias_Theta  = mean(Theta)