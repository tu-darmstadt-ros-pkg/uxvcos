function Kalmangain(block)

setup(block);

%endfunction

function setup(block)

%% Register dialog parameter: LMS step size

block.NumDialogPrms = 3;
block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable'};

% Set up the continuous states.
block.NumContStates = 0;

%% Regieste number of input and output ports
block.NumInputPorts  = 4;
block.NumOutputPorts = 2;

%% Setup functional port properties to dynamically
%% inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% x_pred
block.InputPort(1).Complexity   = 'Real';
block.InputPort(1).DataTypeId   = 0;
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).Dimensions   = 31;

% P_pred
block.InputPort(2).Complexity   = 'Real';
block.InputPort(2).DataTypeId   = 0;
block.InputPort(2).SamplingMode = 'Sample';
block.InputPort(2).Dimensions   = [31 31];

% measurements
block.InputPort(3).Complexity   = 'Real';
block.InputPort(3).DataTypeId   = 0;
block.InputPort(3).SamplingMode = 'Sample';
block.InputPort(3).Dimensions   = 19;

% sensors
block.InputPort(4).Complexity   = 'Real';
block.InputPort(4).DataTypeId   = 0;
block.InputPort(4).SamplingMode = 'Sample';
block.InputPort(4).Dimensions   = 7;

% x(k)
block.OutputPort(1).Complexity   = 'Real';
block.OutputPort(1).DataTypeId   = 0;
block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(1).Dimensions   = 31;
% P(k)
block.OutputPort(2).Complexity   = 'Real';
block.OutputPort(2).DataTypeId   = 0;
block.OutputPort(2).SamplingMode = 'Sample';
block.OutputPort(2).Dimensions   = [31 31];

% Register the sample times.
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1 0];

% -----------------------------------------------------------------
% Options
% -----------------------------------------------------------------
% Specify if Accelerator should use TLC or call back to the
% M-file
block.SetAccelRunOnTLC(false);

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same SimState as a built-in block
%    'HasNoSimState',   < No SimState
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Errors out when saving or restoring the SimState
block.SimStateCompliance = 'DefaultSimState';

%% Register methods
block.RegBlockMethod('ProcessParameters',             @ProcessPrms);
block.RegBlockMethod('PostPropagationSetup',          @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',          @InitConditions);
block.RegBlockMethod('Outputs',                       @Outputs);
block.RegBlockMethod('Terminate',                     @Terminate);

%endfunction

function DoPostPropSetup(block)

%% Setup Dwork
block.NumDworks                = 1;
block.Dwork(1).Name            = 'Pos0';
block.Dwork(1).Dimensions      = [2 1];
block.Dwork(1).DatatypeID      = 0;
block.Dwork(1).Complexity      = 'Real';
block.Dwork(1).UsedAsDiscState = true;

%% Register all tunable parameters as runtime parameters.
block.AutoRegRuntimePrms;

%endfunction

function ProcessPrms(block)

block.AutoUpdateRuntimePrms;

% %endfunction


function InitConditions(block)

%% Initialize Dwork
block.Dwork(1).Data      = zeros(2,1);               % Pos_0
block.OutputPort(1).Data = block.DialogPrm(1).Data;  % x_0
block.OutputPort(2).Data = block.DialogPrm(5).Data;  % P_0

%endfunction

function Outputs(block)

xpred               = block.InputPort(1).Data;
Ppred               = block.InputPort(2).Data;
all_measurements    = block.InputPort(3).Data;
sensors             = block.InputPort(4).Data;

R0                  = block.DialogPrm(1).Data;
B_fix               = block.DialogPrm(2).Data;
parameter           = block.DialogPrm(3).Data;
Pos0                = [0 0];

% Horizontal GPS Position
if sensors(1) == 1
    if block.Dwork(1).Data(1) == 0
        block.Dwork(1).Data = all_measurements(1:2);
        Pos0 = block.Dwork(1).Data;
    end
    [temp, C, R] = getGPSPosition(xpred,all_measurements(1:2),R0(1:2),Pos0);
    K            = Ppred*C'*(C*Ppred*C' + R)^(-1);
    xpred        = xpred + K*temp;
    Ppred        = (eye(length(xpred)) - K*C)*Ppred;
end
% Barometric height
if sensors(2) == 1
    [temp,C,R]  =  getBarometer(xpred,all_measurements(3),R0(3));
    K            = Ppred*C'*(C*Ppred*C' + R)^(-1);
    xpred        = xpred + K*temp;
    Ppred        = (eye(length(xpred)) - K*C)*Ppred;
end
% GPS Velocity
if sensors(3) == 1
    [temp,C,R]  =  getGPSVelocity(xpred,all_measurements(4:6),R0(4:6));
    K            = Ppred*C'*(C*Ppred*C' + R)^(-1);
    xpred        = xpred + K*temp;
    Ppred        = (eye(length(xpred)) - K*C)*Ppred;
end
% Acceleration
if sensors(4) == 1
    [temp,C,R]  =  getAccelerometer(xpred,all_measurements(7:9),R0(7:9),parameter);
    K            = Ppred*C'*(C*Ppred*C' + R)^(-1);
    xpred        = xpred + K*temp;
    Ppred        = (eye(length(xpred)) - K*C)*Ppred;
end
% Gyroscope
if sensors(5) == 1
    [temp,C,R]  =  getGyroscope(xpred,all_measurements(10:12),R0(10:12));
    K            = Ppred*C'*(C*Ppred*C' + R)^(-1);
    xpred        = xpred + K*temp;
    Ppred        = (eye(length(xpred)) - K*C)*Ppred;
end
% Omega motor
if sensors(6) == 1
    [temp,C,R]  =  getOmegaMotor(xpred,all_measurements(13:16),R0(13:16));
    K            = Ppred*C'*(C*Ppred*C' + R)^(-1);
    xpred        = xpred + K*temp;
    Ppred        = (eye(length(xpred)) - K*C)*Ppred;
end
% Magnetometer
if sensors(7) == 1
    [temp, C, R] = getMagneticField(xpred,all_measurements(17:19),R0(17:19),B_fix);
    K            = Ppred*C'*(C*Ppred*C' + R)^(-1);
    xpred        = xpred + K*temp;
    Ppred        = (eye(length(xpred)) - K*C)*Ppred;
end
% normalize quaternions
q_0 = xpred(7);
q_1 = xpred(8);
q_2 = xpred(9);
q_3 = xpred(10);
q_length = sqrt((q_0*q_0)+(q_1*q_1)+(q_2*q_2)+(q_3*q_3));

xpred(7)  = xpred(7)/q_length;
xpred(8)  = xpred(8)/q_length;
xpred(9)  = xpred(9)/q_length;
xpred(10) = xpred(10)/q_length;

block.OutputPort(1).Data = xpred;
block.OutputPort(2).Data = Ppred;

%endfunction

function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

%endfunction

function [temp, C, R] = getGPSPosition(x,real_measurements,Rin,Pos0)

y = zeros(2,1);
C = zeros(2,31);
R = zeros(2);

y(1)   = Pos0(1) + x(1)/6378137.0/pi*180.0;                         % Latitude
y(2)   = Pos0(2) + x(2)/6378137.0/pi/cos(Pos0(1)*pi/180.0)*180.0;   % Longitude

C(1,1) = 180/6378137/pi;
C(2,2) = 180/6378137/pi/cos(Pos0(1)*pi/180);

R(1,1) = 180*Rin(1)/6378137/pi;
R(2,2) = 180*Rin(2)/6378137/pi/cos(Pos0(1)*pi/180);

temp   = real_measurements - y;

function [temp, C, R] = getGPSVelocity(xin,real_measurements,Rin)

y = zeros(3,1);
C = zeros(3,31);

u = xin(4);
v = xin(5);
w = xin(6);
q_0 = xin(7);
q_1 = xin(8);
q_2 = xin(9);
q_3 = xin(10);

y(1) = u*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)-v*(q_0*q_3*2.0-q_1*q_2*2.0)+w*(q_0*q_2*2.0+q_1*q_3*2.0);
y(2) = v*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)+u*(q_0*q_3*2.0+q_1*q_2*2.0)-w*(q_0*q_1*2.0-q_2*q_3*2.0);
y(3) = w*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3)-u*(q_0*q_2*2.0-q_1*q_3*2.0)+v*(q_0*q_1*2.0+q_2*q_3*2.0);

C(1,4)  = q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3;
C(1,5)  = q_0*q_3*-2.0+q_1*q_2*2.0;
C(1,6)  = q_0*q_2*2.0+q_1*q_3*2.0;
C(1,7)  = q_0*u*2.0-q_3*v*2.0+q_2*w*2.0;
C(1,8)  = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
C(1,9)  = q_2*u*-2.0+q_1*v*2.0+q_0*w*2.0;
C(1,10) = q_3*u*-2.0-q_0*v*2.0+q_1*w*2.0;
C(2,4)  = q_0*q_3*2.0+q_1*q_2*2.0;
C(2,5)  = q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3;
C(2,6)  = q_0*q_1*-2.0+q_2*q_3*2.0;
C(2,7)  = q_3*u*2.0+q_0*v*2.0-q_1*w*2.0;
C(2,8)  = q_2*u*2.0-q_1*v*2.0-q_0*w*2.0;
C(2,9)  = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
C(2,10) = q_0*u*2.0-q_3*v*2.0+q_2*w*2.0;
C(3,4)  = q_0*q_2*-2.0+q_1*q_3*2.0;
C(3,5)  = q_0*q_1*2.0+q_2*q_3*2.0;
C(3,6)  = q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3;
C(3,7)  = q_2*u*-2.0+q_1*v*2.0+q_0*w*2.0;
C(3,8)  = q_3*u*2.0+q_0*v*2.0-q_1*w*2.0;
C(3,9)  = q_0*u*-2.0+q_3*v*2.0-q_2*w*2.0;
C(3,10) = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;

R       = diag(Rin);

temp    = real_measurements - y;


function [temp, C, R] = getMagneticField(xin,real_measurements,Rin,B)

q_0 = xin(7);
q_1 = xin(8);
q_2 = xin(9);
q_3 = xin(10);

y       = zeros(3,1);
C       = zeros(3,31);

fieldLength = sqrt(eps + B(1)*B(1)+B(2)*B(2)+B(3)*B(3));

%% Magnetometer updates only heading angle
y(1) = B(2)*(q_0*q_3*2.0+q_1*q_2*2.0)*1.0/fieldLength-B(3)*(q_0*q_2*2.0-q_1*q_3*2.0)*1.0/fieldLength+B(1)*1.0/fieldLength*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3);
y(2) = -B(1)*(q_0*q_3*2.0-q_1*q_2*2.0)*1.0/fieldLength+B(3)*(q_0*q_1*2.0+q_2*q_3*2.0)*1.0/fieldLength+B(2)*1.0/fieldLength*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3);
y(3) = B(1)*(q_0*q_2*2.0+q_1*q_3*2.0)*1.0/fieldLength-B(2)*(q_0*q_1*2.0-q_2*q_3*2.0)*1.0/fieldLength+B(3)*1.0/fieldLength*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3);

C(1,7) = -q_3*(q_0*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_1*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_2*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)-q_3*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0));
C(1,8) = q_2*(q_0*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_1*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_2*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)-q_3*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0));
C(1,9) = -q_1*(q_0*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_1*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_2*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)-q_3*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0));
C(1,10) = q_0*(q_0*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_1*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_2*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)-q_3*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0));
C(2,7) = q_3*(q_0*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)-q_2*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_1*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_3*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0));
C(2,8) = -q_2*(q_0*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)-q_2*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_1*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_3*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0));
C(2,9) = q_1*(q_0*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)-q_2*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_1*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_3*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0));
C(2,10) = -q_0*(q_0*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)-q_2*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0)+q_1*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_3*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0));
C(3,7) = q_3*(-q_0*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_1*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)+q_2*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_3*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0));
C(3,8) = -q_2*(-q_0*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_1*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)+q_2*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_3*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0));
C(3,9) = q_1*(-q_0*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_1*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)+q_2*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_3*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0));
C(3,10) = -q_0*(-q_0*(B(1)*q_1*1.0/fieldLength*2.0+B(2)*q_2*1.0/fieldLength*2.0+B(3)*q_3*1.0/fieldLength*2.0)+q_1*(B(1)*q_0*1.0/fieldLength*2.0+B(2)*q_3*1.0/fieldLength*2.0-B(3)*q_2*1.0/fieldLength*2.0)+q_2*(B(2)*q_0*1.0/fieldLength*2.0-B(1)*q_3*1.0/fieldLength*2.0+B(3)*q_1*1.0/fieldLength*2.0)+q_3*(B(1)*q_2*1.0/fieldLength*2.0-B(2)*q_1*1.0/fieldLength*2.0+B(3)*q_0*1.0/fieldLength*2.0));

R       = diag(Rin);

temp    = real_measurements - y;


function [temp,C,R] = getBarometer(x,real_measurements,Rin)

%% Initial conditions
p_z = x(3);

y(1)   = p_z;
C      = zeros(1,31);

C(1,3) = 1.0;

R       = diag(Rin);

temp    = real_measurements - y;

function [temp,C,R] = getAccelerometer(x,real_measurements,Rin,parameter)

% parameters
g     = parameter.g;
l_m   = parameter.l_m;
m     = parameter.m;
CT2s  = parameter.CT2s;
CT1s  = parameter.CT1s;
C_wz  = parameter.C_wz;

%% Initial conditions
u = x(4);
v = x(5);
w = x(6);
q_0 = x(7);
q_1 = x(8);
q_2 = x(9);
q_3 = x(10);
a_x_bias = x(11);
a_y_bias = x(12);
a_z_bias = x(13);

p = x(17);
q = x(18);
r = x(19);
w_1 = x(20);
w_2 = x(21);
w_3 = x(22);
w_4 = x(23);

CT0s = x(25);

v_x_wind = x(29);
v_y_wind = x(30);
C_wxy    = x(31);

y     = zeros(3,1);
C     = zeros(3,31);

if sum(x(20:23)) < 1500 % ground contact if motor speed is to low
    y(1) = a_x_bias+g*(q_0*q_2*2.0-q_1*q_3*2.0);
    y(2) = a_y_bias-g*(q_0*q_1*2.0+q_2*q_3*2.0);
    y(3) = a_z_bias-g*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3);
    
    C(1,7) = g*q_2*2.0;
    C(1,8) = g*q_3*-2.0;
    C(1,9) = g*q_0*2.0;
    C(1,10) = g*q_1*-2.0;
    C(1,11) = 1.0;
    C(2,7) = g*q_1*-2.0;
    C(2,8) = g*q_0*-2.0;
    C(2,9) = g*q_3*-2.0;
    C(2,10) = g*q_2*-2.0;
    C(2,12) = 1.0;
    C(3,7) = g*q_0*-2.0;
    C(3,8) = g*q_1*2.0;
    C(3,9) = g*q_2*2.0;
    C(3,10) = g*q_3*-2.0;
    C(3,13) = 1.0;
else
    vx_res = -u+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0);
    vy_res = v-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0);
    vz_res = w-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0);
    velocityLength = sqrt(eps + vx_res^2 + vy_res^2 + vz_res^2);
    
    y(1) = a_x_bias-q*w+r*v+(C_wxy*(vx_res)*velocityLength)/m;
    y(2) = a_y_bias+p*w-r*u-(C_wxy*velocityLength*(vy_res))/m;
    y(3) = a_z_bias-p*v+q*u-(CT0s*(w_1*w_1)+CT0s*(w_2*w_2)+CT0s*(w_3*w_3)+CT0s*(w_4*w_4)-CT1s*w_2*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT1s*w_3*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*w_4*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT1s*w_1*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(vz_res)*velocityLength)/m;
    
    C(1,4) = -(C_wxy*velocityLength)/m-(C_wxy*(vx_res)*(u*-2.0+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(1,5) = r+(C_wxy*(vx_res)*(v*2.0-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(1,6) = -q+(C_wxy*(vx_res)*(w*2.0-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(1,7) = (C_wxy*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*velocityLength)/m+(C_wxy*(vx_res)*((q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vy_res)*-2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vx_res)*2.0+(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(1,8) = (C_wxy*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*velocityLength)/m+(C_wxy*(vx_res)*((q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vy_res)*2.0+(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vx_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(1,9) = (C_wxy*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*velocityLength)/m-(C_wxy*(vx_res)*((q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vy_res)*2.0-(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vx_res)*2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(1,10) = (C_wxy*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*velocityLength)/m+(C_wxy*(vx_res)*((q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vy_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vx_res)*2.0-(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(1,11) = 1.0;
    C(1,18) = -w;
    C(1,19) = v;
    C(1,29) = (C_wxy*velocityLength*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3))/m+(C_wxy*(vx_res)*1.0/velocityLength*((q_0*q_3*2.0-q_1*q_2*2.0)*(vy_res)*2.0-(q_0*q_2*2.0+q_1*q_3*2.0)*(vz_res)*2.0+(vx_res)*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0)*(1.0/2.0))/m;
    C(1,30) = (C_wxy*(q_0*q_3*2.0+q_1*q_2*2.0)*velocityLength)/m+(C_wxy*(vx_res)*1.0/velocityLength*((q_0*q_3*2.0+q_1*q_2*2.0)*(vx_res)*2.0-(vy_res)*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+(q_0*q_1*2.0-q_2*q_3*2.0)*(vz_res)*2.0)*(1.0/2.0))/m;
    C(1,31) = ((vx_res)*velocityLength)/m;
    C(2,4) = -r+(C_wxy*(u*-2.0+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
    C(2,5) = -(C_wxy*velocityLength)/m-(C_wxy*(v*2.0-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
    C(2,6) = p-(C_wxy*(w*2.0-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
    C(2,7) = (C_wxy*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*velocityLength)/m-(C_wxy*((q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vy_res)*-2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vx_res)*2.0+(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
    C(2,8) = -(C_wxy*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*velocityLength)/m-(C_wxy*((q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vy_res)*2.0+(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vx_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
    C(2,9) = (C_wxy*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*velocityLength)/m+(C_wxy*((q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vy_res)*2.0-(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vx_res)*2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
    C(2,10) = -(C_wxy*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*velocityLength)/m-(C_wxy*((q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vy_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vx_res)*2.0-(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
    C(2,12) = 1.0;
    C(2,17) = w;
    C(2,19) = -u;
    C(2,29) = -(C_wxy*(q_0*q_3*2.0-q_1*q_2*2.0)*velocityLength)/m-(C_wxy*1.0/velocityLength*(vy_res)*((q_0*q_3*2.0-q_1*q_2*2.0)*(vy_res)*2.0-(q_0*q_2*2.0+q_1*q_3*2.0)*(vz_res)*2.0+(vx_res)*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0)*(1.0/2.0))/m;
    C(2,30) = (C_wxy*velocityLength*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3))/m-(C_wxy*1.0/velocityLength*(vy_res)*((q_0*q_3*2.0+q_1*q_2*2.0)*(vx_res)*2.0-(vy_res)*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+(q_0*q_1*2.0-q_2*q_3*2.0)*(vz_res)*2.0)*(1.0/2.0))/m;
    C(2,31) = -(velocityLength*(vy_res))/m;
    C(3,4) = q+(C_wz*(u*-2.0+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0)*2.0)*(vz_res)*1.0/velocityLength*(1.0/2.0))/m;
    C(3,5) = -p-(C_wz*(v*2.0-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0)*2.0)*(vz_res)*1.0/velocityLength*(1.0/2.0))/m;
    C(3,6) = (CT1s*w_1+CT1s*w_2+CT1s*w_3+CT1s*w_4+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*velocityLength)/m-(C_wz*(vz_res)*(w*2.0-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(3,7) = (CT1s*w_1*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT1s*w_2*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT1s*w_3*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT1s*w_4*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*velocityLength)/m-(C_wz*(vz_res)*((q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vy_res)*-2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vx_res)*2.0+(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(3,8) = (CT1s*w_1*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT1s*w_2*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT1s*w_3*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT1s*w_4*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*velocityLength)/m-(C_wz*(vz_res)*((q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vy_res)*2.0+(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vx_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(3,9) = -(CT1s*w_1*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT1s*w_2*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT1s*w_3*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT1s*w_4*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m+(C_wz*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*velocityLength)/m+(C_wz*(vz_res)*((q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vy_res)*2.0-(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vx_res)*2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(3,10) = -(CT1s*w_1*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT1s*w_2*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT1s*w_3*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT1s*w_4*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m+(C_wz*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*velocityLength)/m-(C_wz*(vz_res)*((q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vy_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vx_res)*2.0-(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
    C(3,13) = 1.0;
    C(3,17) = -v+(CT2s*l_m*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*l_m*w_2-CT1s*l_m*w_4-CT2s*l_m*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*l_m*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*l_m*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
    C(3,18) = u+(CT2s*l_m*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT1s*l_m*w_1+CT1s*l_m*w_3-CT2s*l_m*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*l_m*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*l_m*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
    C(3,20) = -(CT0s*w_1*2.0+CT1s*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0))/m;
    C(3,21) = -(CT0s*w_2*2.0-CT1s*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
    C(3,22) = -(CT0s*w_3*2.0-CT1s*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
    C(3,23) = -(CT0s*w_4*2.0+CT1s*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0))/m;
    C(3,25) = -(w_1*w_1+w_2*w_2+w_3*w_3+w_4*w_4)/m;
    C(3,29) = -(CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_1*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_2*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_3*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_4*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m+(C_wz*(q_0*q_2*2.0+q_1*q_3*2.0)*velocityLength)/m-(C_wz*(vz_res)*1.0/velocityLength*((q_0*q_3*2.0-q_1*q_2*2.0)*(vy_res)*2.0-(q_0*q_2*2.0+q_1*q_3*2.0)*(vz_res)*2.0+(vx_res)*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0)*(1.0/2.0))/m;
    C(3,30) = (CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_1*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_2*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_3*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_4*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(q_0*q_1*2.0-q_2*q_3*2.0)*velocityLength)/m-(C_wz*(vz_res)*1.0/velocityLength*((q_0*q_3*2.0+q_1*q_2*2.0)*(vx_res)*2.0-(vy_res)*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+(q_0*q_1*2.0-q_2*q_3*2.0)*(vz_res)*2.0)*(1.0/2.0))/m;
end
R        = diag(Rin);

temp = real_measurements - y;

function [temp,C,R] = getGyroscope(x,real_measurements,Rin)

%% Initial conditions
p_bias = x(14);
q_bias = x(15);
r_bias = x(16);
p = x(17);
q = x(18);
r = x(19);

y   = zeros(3,1);
C   = zeros(3,31);

y(1) = p+p_bias;
y(2) = q+q_bias;
y(3) = r+r_bias;

C(1,14) = 1.0;
C(1,17) = 1.0;
C(2,15) = 1.0;
C(2,18) = 1.0;
C(3,16) = 1.0;
C(3,19) = 1.0;

R       = diag(Rin);

temp = real_measurements - y;

function [temp,C,R] = getOmegaMotor(x,real_measurements,Rin)

%% Initial conditions

w_1 = x(20);
w_2 = x(21);
w_3 = x(22);
w_4 = x(23);

y    = zeros(4,1);
C    = zeros(4,31);

y(1) = w_1;
y(2) = w_2;
y(3) = w_3;
y(4) = w_4;

C(1,20) = 1.0;
C(2,21) = 1.0;
C(3,22) = 1.0;
C(4,23) = 1.0;

R       = diag(Rin);

temp = real_measurements - y;
