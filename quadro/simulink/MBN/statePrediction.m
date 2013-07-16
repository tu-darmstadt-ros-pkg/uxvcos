function statePrediction(block)

setup(block);

%endfunction

function setup(block)

%% Register dialog parameter: LMS step size

block.NumDialogPrms = 5;
block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Nontunable','Nontunable'};

% Set up the continuous states.
block.NumContStates = 0;

%% Register number of input and output ports
block.NumInputPorts  = 4;
block.NumOutputPorts = 2;

%% Setup functional port properties to dynamically inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
% u
block.InputPort(1).Complexity   = 'Real';
block.InputPort(1).DataTypeId   = 0;
block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(1).Dimensions   = 4;
% x(k-1)
block.InputPort(2).Complexity   = 'Real';
block.InputPort(2).DataTypeId   = 0;
block.InputPort(2).SamplingMode = 'Sample';
block.InputPort(2).Dimensions   = 31;
% P(k-1)
block.InputPort(3).Complexity   = 'Real';
block.InputPort(3).DataTypeId   = 0;
block.InputPort(3).SamplingMode = 'Sample';
block.InputPort(3).Dimensions   = [31 31];

% dt
block.InputPort(4).Complexity   = 'Real';
block.InputPort(4).DataTypeId   = 0;
block.InputPort(4).SamplingMode = 'Sample';
block.InputPort(4).Dimensions   = [1];

% x_pred
block.OutputPort(1).Complexity   = 'Real';
block.OutputPort(1).DataTypeId   = 0;
block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(1).Dimensions   = 31;

% P_pred
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
block.RegBlockMethod('CheckParameters',               @CheckPrms);
block.RegBlockMethod('ProcessParameters',             @ProcessPrms);
block.RegBlockMethod('PostPropagationSetup',          @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',          @InitConditions);
block.RegBlockMethod('Derivatives',                   @Derivatives);
block.RegBlockMethod('Outputs',                       @Outputs);
block.RegBlockMethod('Terminate',                     @Terminate);

%endfunction

function CheckPrms(block)
x_0 = block.DialogPrm(1).Data;

if length(x_0) ~= block.OutputPort(1).Dimensions
    error(['length of x0 has to be ' num2str(block.OutputPort(1).Dimensions)]);
end

%endfunction

function DoPostPropSetup(block)

  %% Setup Dwork
  block.NumDworks                = 1;
  block.Dwork(1).Name            = 'u'; 
  block.Dwork(1).Dimensions      = [4 1];
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
parameter                = block.DialogPrm(2).Data;
block.Dwork(1).Data      = ones(4,1)*parameter.U0;   % u_0
block.OutputPort(1).Data = block.DialogPrm(1).Data;  % x_0
block.OutputPort(2).Data = block.DialogPrm(5).Data;  % P_0

%endfunction

function Outputs(block)

u         = block.InputPort(1).Data;
x_old     = block.InputPort(2).Data;
P         = block.InputPort(3).Data;
dt        = block.InputPort(4).Data;

parameter = block.DialogPrm(2).Data;
Qin       = block.DialogPrm(3).Data;

if x_old == block.DialogPrm(1).Data
    u     = ones(4,1)*parameter.U0;
end

% Euler step
% dx = equationsOfMotion(x_old, u, parameter);
% x  = x_old + dt*dx;

% Heun Step
if sum(block.Dwork(1).Data) > 12 
    K1 = equationsOfMotion(x_old, block.Dwork(1).Data, parameter);
    K2 = equationsOfMotion(x_old+dt*K1, u, parameter);
else
    K1 = zeros(size(x_old));
    K2 = K1;
end

x  = x_old + dt*(K1+K2)/2;

% Bogacki-Shampine 3rd Order
% k1 = equationsOfMotion(x_old,block.Dwork(1).Data, parameter);             % euler/heun/bs 1ter schritt
% k2 = equationsOfMotion(x_old+0.50*dt*k1, 1/2*(block.Dwork(1).Data + u), parameter);
% k3 = equationsOfMotion(x_old+0.75*dt*k2, 1/4*(block.Dwork(1).Data + 3*u), parameter);
% 
% x  = x_old + dt*(2*k1+3*k2+4*k3)/9;

[A,Q] = systemJacobians(x, u, Qin, parameter, dt);

block.Dwork(1).Data = u;
% prediction of x
block.OutputPort(1).Data = x;
% prediction of P
block.OutputPort(2).Data = A*P*A' + Q;
%endfunction


function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

%endfunction

function fx = equationsOfMotion(x, uin, parameter)

% parameters
g     = parameter.g;  
m     = parameter.m;
l_m   = parameter.l_m; 
I_x   = parameter.I_x;
I_y   = parameter.I_y;
I_z   = parameter.I_z; 
Psi   = parameter.Psi; 
R_A   = parameter.R_A;
J_M   = parameter.J_M;
CT2s  = parameter.CT2s;
CT1s  = parameter.CT1s;
% C_wxy = x(28);%parameter.C_wxy; 
C_wz  = parameter.C_wz;
C_mxy = parameter.C_mxy;
C_mz  = parameter.C_mz;
alpha_m = parameter.alpha_m;
beta_m  = parameter.beta_m;
k_m  = parameter.k_m;

 % temporarily used Expressions
U_1 = uin(1);
U_2 = uin(2);
U_3 = uin(3);
U_4 = uin(4);

p_x = x(1);
p_y = x(2);
p_z = x(3);
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
p_bias = x(14);
q_bias = x(15);
r_bias = x(16);
p = x(17);
q = x(18);
r = x(19);
w_1 = x(20);
w_2 = x(21);
w_3 = x(22);
w_4 = x(23);
k_t = x(24);
CT0s = x(25);
M_x0 = x(26);
M_y0 = x(27);
M_z0 = x(28);
v_x_wind = x(29);
v_y_wind = x(30);
C_wxy = x(31);

vx_res = -u+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0);
vy_res = v-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0);
vz_res = w-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0);
velocityLength = sqrt(eps + vx_res^2 + vy_res^2 + vz_res^2);

 % DGLn berechnen
fx = zeros(31,1);

fx(1) = u*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)-v*(q_0*q_3*2.0-q_1*q_2*2.0)+w*(q_0*q_2*2.0+q_1*q_3*2.0);
fx(2) = v*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)+u*(q_0*q_3*2.0+q_1*q_2*2.0)-w*(q_0*q_1*2.0-q_2*q_3*2.0);
fx(3) = w*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3)-u*(q_0*q_2*2.0-q_1*q_3*2.0)+v*(q_0*q_1*2.0+q_2*q_3*2.0);
fx(4) = -q*w+r*v-g*(q_0*q_2*2.0-q_1*q_3*2.0)+(C_wxy*(vx_res)*velocityLength)/m;
fx(5) = p*w-r*u+g*(q_0*q_1*2.0+q_2*q_3*2.0)-(C_wxy*velocityLength*(vy_res))/m;
fx(6) = g*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3)-p*v+q*u-(CT0s*(w_1*w_1)+CT0s*(w_2*w_2)+CT0s*(w_3*w_3)+CT0s*(w_4*w_4)-CT1s*w_2*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT1s*w_3*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*w_4*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT1s*w_1*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(vz_res)*velocityLength)/m;
fx(7) = p*q_1*(-1.0/2.0)-q*q_2*(1.0/2.0)-q_3*r*(1.0/2.0)-q_0*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
fx(8) = p*q_0*(1.0/2.0)-q*q_3*(1.0/2.0)+q_2*r*(1.0/2.0)-q_1*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
fx(9) = p*q_3*(1.0/2.0)+q*q_0*(1.0/2.0)-q_1*r*(1.0/2.0)-q_2*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
fx(10) = p*q_2*(-1.0/2.0)+q*q_1*(1.0/2.0)+q_0*r*(1.0/2.0)-q_3*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
fx(17) = (M_x0-C_mxy*p*sqrt(p*p+q*q+r*r)+I_y*q*r-I_z*q*r-CT0s*l_m*(w_2*w_2)+CT0s*l_m*(w_4*w_4)+CT2s*(l_m*l_m)*p*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT2s*l_m*w*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT1s*(l_m*l_m)*p*w_2+CT1s*(l_m*l_m)*p*w_4+CT2s*(l_m*l_m)*p*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*l_m*w*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*l_m*w*w_2-CT1s*l_m*w*w_4-CT2s*l_m*q_0*q_1*v_y_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_1*v_y_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_0*q_2*v_x_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_1*q_3*v_x_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*q_3*v_y_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT1s*l_m*q_0*q_1*v_y_wind*w_2*2.0-CT1s*l_m*q_0*q_2*v_x_wind*w_2*2.0-CT1s*l_m*q_0*q_1*v_y_wind*w_4*2.0+CT1s*l_m*q_0*q_2*v_x_wind*w_4*2.0-CT1s*l_m*q_1*q_3*v_x_wind*w_2*2.0+CT1s*l_m*q_1*q_3*v_x_wind*w_4*2.0-CT1s*l_m*q_2*q_3*v_y_wind*w_2*2.0+CT1s*l_m*q_2*q_3*v_y_wind*w_4*2.0)/I_x;
fx(18) = (M_y0-C_mxy*q*sqrt(p*p+q*q+r*r)-I_x*p*r+I_z*p*r+CT0s*l_m*(w_1*w_1)-CT0s*l_m*(w_3*w_3)+CT2s*(l_m*l_m)*q*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT2s*l_m*w*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT1s*(l_m*l_m)*q*w_1+CT1s*(l_m*l_m)*q*w_3+CT2s*(l_m*l_m)*q*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*l_m*w*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT1s*l_m*w*w_1+CT1s*l_m*w*w_3-CT2s*l_m*q_0*q_1*v_y_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_1*v_y_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_0*q_2*v_x_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_1*q_3*v_x_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*q_3*v_y_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT1s*l_m*q_0*q_1*v_y_wind*w_1*2.0+CT1s*l_m*q_0*q_2*v_x_wind*w_1*2.0+CT1s*l_m*q_0*q_1*v_y_wind*w_3*2.0-CT1s*l_m*q_0*q_2*v_x_wind*w_3*2.0+CT1s*l_m*q_1*q_3*v_x_wind*w_1*2.0-CT1s*l_m*q_1*q_3*v_x_wind*w_3*2.0+CT1s*l_m*q_2*q_3*v_y_wind*w_1*2.0-CT1s*l_m*q_2*q_3*v_y_wind*w_3*2.0)/I_y;
fx(19) = ((Psi*Psi)*w_1*(1.0/2.0)-(Psi*Psi)*w_2*(1.0/2.0)+(Psi*Psi)*w_3*(1.0/2.0)-(Psi*Psi)*w_4*(1.0/2.0)+M_z0*R_A-Psi*((R_A/abs(R_A)))*sqrt((U_1*U_1)*(beta_m*beta_m)+(Psi*Psi)*(w_1*w_1)+R_A*U_1*alpha_m*4.0-Psi*U_1*beta_m*w_1*2.0)*(1.0/2.0)+Psi*((R_A/abs(R_A)))*sqrt((U_2*U_2)*(beta_m*beta_m)+(Psi*Psi)*(w_2*w_2)+R_A*U_2*alpha_m*4.0-Psi*U_2*beta_m*w_2*2.0)*(1.0/2.0)-Psi*((R_A/abs(R_A)))*sqrt((U_3*U_3)*(beta_m*beta_m)+(Psi*Psi)*(w_3*w_3)+R_A*U_3*alpha_m*4.0-Psi*U_3*beta_m*w_3*2.0)*(1.0/2.0)+Psi*((R_A/abs(R_A)))*sqrt((U_4*U_4)*(beta_m*beta_m)+(Psi*Psi)*(w_4*w_4)+R_A*U_4*alpha_m*4.0-Psi*U_4*beta_m*w_4*2.0)*(1.0/2.0)-Psi*U_1*beta_m*(1.0/2.0)+Psi*U_2*beta_m*(1.0/2.0)-Psi*U_3*beta_m*(1.0/2.0)+Psi*U_4*beta_m*(1.0/2.0)-C_mz*R_A*r*sqrt(p*p+q*q+r*r)+I_x*R_A*p*q-I_y*R_A*p*q)/(I_z*R_A);
fx(20) = (-k_m*w_1+Psi*(sqrt(1.0/(R_A*R_A)*power(U_1*beta_m-Psi*w_1,2.0)*(1.0/4.0)+(U_1*alpha_m)/R_A)+(U_1*beta_m*(1.0/2.0)-Psi*w_1*(1.0/2.0))/R_A)+k_t*(-CT0s*(w_1*w_1)+CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_1*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
fx(21) = (-k_m*w_2+k_t*(-CT0s*(w_2*w_2)+CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_2*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)))+Psi*(sqrt(1.0/(R_A*R_A)*power(U_2*beta_m-Psi*w_2,2.0)*(1.0/4.0)+(U_2*alpha_m)/R_A)+(U_2*beta_m*(1.0/2.0)-Psi*w_2*(1.0/2.0))/R_A))/J_M;
fx(22) = (-k_m*w_3+Psi*(sqrt(1.0/(R_A*R_A)*power(U_3*beta_m-Psi*w_3,2.0)*(1.0/4.0)+(U_3*alpha_m)/R_A)+(U_3*beta_m*(1.0/2.0)-Psi*w_3*(1.0/2.0))/R_A)+k_t*(-CT0s*(w_3*w_3)+CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_3*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
fx(23) = (-k_m*w_4+k_t*(-CT0s*(w_4*w_4)+CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_4*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)))+Psi*(sqrt(1.0/(R_A*R_A)*power(U_4*beta_m-Psi*w_4,2.0)*(1.0/4.0)+(U_4*alpha_m)/R_A)+(U_4*beta_m*(1.0/2.0)-Psi*w_4*(1.0/2.0))/R_A))/J_M;

% % old version without wind speed
%  % DGLn berechnen
% fx = zeros(28,1);
% 
% fx(1) = u*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)-v*(q_0*q_3*2.0-q_1*q_2*2.0)+w*(q_0*q_2*2.0+q_1*q_3*2.0);
% fx(2) = v*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)+u*(q_0*q_3*2.0+q_1*q_2*2.0)-w*(q_0*q_1*2.0-q_2*q_3*2.0);
% fx(3) = w*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3)-u*(q_0*q_2*2.0-q_1*q_3*2.0)+v*(q_0*q_1*2.0+q_2*q_3*2.0);
% fx(4) = -q*w+r*v-g*(q_0*q_2*2.0-q_1*q_3*2.0)-(C_wxy*u*sqrt(u*u+v*v+w*w))/m;
% fx(5) = p*w-r*u+g*(q_0*q_1*2.0+q_2*q_3*2.0)-(C_wxy*v*sqrt(u*u+v*v+w*w))/m;
% fx(6) = g*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3)-p*v+q*u+(-CT0s*(w_1*w_1)-CT0s*(w_2*w_2)-CT0s*(w_3*w_3)-CT0s*(w_4*w_4)+CT2s*abs(w+l_m*p)*(w+l_m*p)+CT2s*abs(w-l_m*p)*(w-l_m*p)+CT2s*abs(w+l_m*q)*(w+l_m*q)+CT2s*abs(w-l_m*q)*(w-l_m*q)+CT1s*w_2*(w+l_m*p)+CT1s*w_4*(w-l_m*p)+CT1s*w_1*(w-l_m*q)+CT1s*w_3*(w+l_m*q))/m-(C_wz*w*sqrt(u*u+v*v+w*w))/m;
% fx(7) = p*q_1*(-1.0/2.0)-q*q_2*(1.0/2.0)-q_3*r*(1.0/2.0)-q_0*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
% fx(8) = p*q_0*(1.0/2.0)-q*q_3*(1.0/2.0)+q_2*r*(1.0/2.0)-q_1*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
% fx(9) = p*q_3*(1.0/2.0)+q*q_0*(1.0/2.0)-q_1*r*(1.0/2.0)-q_2*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
% fx(10) = p*q_2*(-1.0/2.0)+q*q_1*(1.0/2.0)+q_0*r*(1.0/2.0)-q_3*((q_0*q_0)*3.0+(q_1*q_1)*3.0+(q_2*q_2)*3.0+(q_3*q_3)*3.0-3.0);
% fx(17) = (M_x0-C_mxy*p*sqrt(p*p+q*q+r*r)+I_y*q*r-I_z*q*r-CT0s*l_m*(w_2*w_2)+CT0s*l_m*(w_4*w_4)+CT2s*(l_m*l_m)*p*abs(w+l_m*p)+CT2s*(l_m*l_m)*p*abs(w-l_m*p)+CT1s*(l_m*l_m)*p*w_2+CT1s*(l_m*l_m)*p*w_4+CT2s*l_m*w*abs(w+l_m*p)-CT2s*l_m*w*abs(w-l_m*p)+CT1s*l_m*w*w_2-CT1s*l_m*w*w_4)/I_x;
% fx(18) = (M_y0-C_mxy*q*sqrt(p*p+q*q+r*r)-I_x*p*r+I_z*p*r+CT0s*l_m*(w_1*w_1)-CT0s*l_m*(w_3*w_3)+CT2s*(l_m*l_m)*q*abs(w+l_m*q)+CT2s*(l_m*l_m)*q*abs(w-l_m*q)+CT1s*(l_m*l_m)*q*w_1+CT1s*(l_m*l_m)*q*w_3+CT2s*l_m*w*abs(w+l_m*q)-CT2s*l_m*w*abs(w-l_m*q)-CT1s*l_m*w*w_1+CT1s*l_m*w*w_3)/I_y;
% fx(19) = ((Psi*Psi)*w_1*(1.0/2.0)-(Psi*Psi)*w_2*(1.0/2.0)+(Psi*Psi)*w_3*(1.0/2.0)-(Psi*Psi)*w_4*(1.0/2.0)+M_z0*R_A-Psi*((R_A/abs(R_A)))*sqrt(( eps + U_1*U_1)*(beta_m*beta_m)+(Psi*Psi)*(w_1*w_1)+R_A*U_1*alpha_m*4.0-Psi*U_1*beta_m*w_1*2.0)*(1.0/2.0)+Psi*((R_A/abs(R_A)))*sqrt(( eps + U_2*U_2)*(beta_m*beta_m)+(Psi*Psi)*(w_2*w_2)+R_A*U_2*alpha_m*4.0-Psi*U_2*beta_m*w_2*2.0)*(1.0/2.0)-Psi*((R_A/abs(R_A)))*sqrt(( eps + U_3*U_3)*(beta_m*beta_m)+(Psi*Psi)*(w_3*w_3)+R_A*U_3*alpha_m*4.0-Psi*U_3*beta_m*w_3*2.0)*(1.0/2.0)+Psi*((R_A/abs(R_A)))*sqrt(( eps + U_4*U_4)*(beta_m*beta_m)+(Psi*Psi)*(w_4*w_4)+R_A*U_4*alpha_m*4.0-Psi*U_4*beta_m*w_4*2.0)*(1.0/2.0)-Psi*U_1*beta_m*(1.0/2.0)+Psi*U_2*beta_m*(1.0/2.0)-Psi*U_3*beta_m*(1.0/2.0)+Psi*U_4*beta_m*(1.0/2.0)-C_mz*R_A*r*sqrt(p*p+q*q+r*r)+I_x*R_A*p*q-I_y*R_A*p*q)/(I_z*R_A);
% fx(20) = (-k_m*w_1+Psi*(sqrt(1.0/(R_A*R_A)*power(U_1*beta_m-Psi*w_1,2.0)*(1.0/4.0)+(U_1*alpha_m)/R_A)+(U_1*beta_m*(1.0/2.0)-Psi*w_1*(1.0/2.0))/R_A)+k_t*(-CT0s*(w_1*w_1)+CT2s*abs(w-l_m*q)*(w-l_m*q)+CT1s*w_1*(w-l_m*q)))/J_M;
% fx(21) = (-k_m*w_2+Psi*(sqrt(1.0/(R_A*R_A)*power(U_2*beta_m-Psi*w_2,2.0)*(1.0/4.0)+(U_2*alpha_m)/R_A)+(U_2*beta_m*(1.0/2.0)-Psi*w_2*(1.0/2.0))/R_A)+k_t*(-CT0s*(w_2*w_2)+CT2s*abs(w+l_m*p)*(w+l_m*p)+CT1s*w_2*(w+l_m*p)))/J_M;
% fx(22) = (-k_m*w_3+Psi*(sqrt(1.0/(R_A*R_A)*power(U_3*beta_m-Psi*w_3,2.0)*(1.0/4.0)+(U_3*alpha_m)/R_A)+(U_3*beta_m*(1.0/2.0)-Psi*w_3*(1.0/2.0))/R_A)+k_t*(-CT0s*(w_3*w_3)+CT2s*abs(w+l_m*q)*(w+l_m*q)+CT1s*w_3*(w+l_m*q)))/J_M;
% fx(23) = (-k_m*w_4+Psi*(sqrt(1.0/(R_A*R_A)*power(U_4*beta_m-Psi*w_4,2.0)*(1.0/4.0)+(U_4*alpha_m)/R_A)+(U_4*beta_m*(1.0/2.0)-Psi*w_4*(1.0/2.0))/R_A)+k_t*(-CT0s*(w_4*w_4)+CT2s*abs(w-l_m*p)*(w-l_m*p)+CT1s*w_4*(w-l_m*p)))/J_M;

function [A,Q] = systemJacobians(x, uin, Qin, parameter, dt)
%%
% parameters
g     = parameter.g;  
m     = parameter.m;
l_m   = parameter.l_m; 
I_x   = parameter.I_x;
I_y   = parameter.I_y;
I_z   = parameter.I_z; 
Psi   = parameter.Psi; 
R_A   = parameter.R_A;
J_M   = parameter.J_M;
CT2s  = parameter.CT2s;
CT1s  = parameter.CT1s;
% C_wxy = parameter.C_wxy; 
C_wz  = parameter.C_wz;
C_mxy = parameter.C_mxy;
C_mz  = parameter.C_mz;
alpha_m = parameter.alpha_m;
beta_m  = parameter.beta_m;
k_m  = parameter.k_m;

 % temporarily used Expressions
U_1 = uin(1);
U_2 = uin(2);
U_3 = uin(3);
U_4 = uin(4);

p_x = x(1);
p_y = x(2);
p_z = x(3);
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
p_bias = x(14);
q_bias = x(15);
r_bias = x(16);
p = x(17);
q = x(18);
r = x(19);
w_1 = x(20);
w_2 = x(21);
w_3 = x(22);
w_4 = x(23);
k_t = x(24);
CT0s = x(25);
M_x0 = x(26);
M_y0 = x(27);
M_z0 = x(28);
v_x_wind = x(29);
v_y_wind = x(30);
C_wxy = x(31);

A = zeros(31);
Q = zeros(31);
W = zeros(3,4);

vx_res = -u+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0);
vy_res = v-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0);
vz_res = w-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0);
velocityLength = sqrt(eps + vx_res^2 + vy_res^2 + vz_res^2);

A(1,4) = q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3;
A(1,5) = q_0*q_3*-2.0+q_1*q_2*2.0;
A(1,6) = q_0*q_2*2.0+q_1*q_3*2.0;
A(1,7) = q_0*u*2.0-q_3*v*2.0+q_2*w*2.0;
A(1,8) = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
A(1,9) = q_2*u*-2.0+q_1*v*2.0+q_0*w*2.0;
A(1,10) = q_3*u*-2.0-q_0*v*2.0+q_1*w*2.0;
A(2,4) = q_0*q_3*2.0+q_1*q_2*2.0;
A(2,5) = q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3;
A(2,6) = q_0*q_1*-2.0+q_2*q_3*2.0;
A(2,7) = q_3*u*2.0+q_0*v*2.0-q_1*w*2.0;
A(2,8) = q_2*u*2.0-q_1*v*2.0-q_0*w*2.0;
A(2,9) = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
A(2,10) = q_0*u*2.0-q_3*v*2.0+q_2*w*2.0;
A(3,4) = q_0*q_2*-2.0+q_1*q_3*2.0;
A(3,5) = q_0*q_1*2.0+q_2*q_3*2.0;
A(3,6) = q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3;
A(3,7) = q_2*u*-2.0+q_1*v*2.0+q_0*w*2.0;
A(3,8) = q_3*u*2.0+q_0*v*2.0-q_1*w*2.0;
A(3,9) = q_0*u*-2.0+q_3*v*2.0-q_2*w*2.0;
A(3,10) = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
A(4,4) = -(C_wxy*velocityLength)/m-(C_wxy*(vx_res)*(u*-2.0+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(4,5) = r+(C_wxy*(vx_res)*(v*2.0-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(4,6) = -q+(C_wxy*(vx_res)*(w*2.0-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(4,7) = g*q_2*-2.0+(C_wxy*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*velocityLength)/m+(C_wxy*(vx_res)*((q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vy_res)*-2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vx_res)*2.0+(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(4,8) = g*q_3*2.0+(C_wxy*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*velocityLength)/m+(C_wxy*(vx_res)*((q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vy_res)*2.0+(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vx_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(4,9) = g*q_0*-2.0+(C_wxy*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*velocityLength)/m-(C_wxy*(vx_res)*((q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vy_res)*2.0-(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vx_res)*2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(4,10) = g*q_1*2.0+(C_wxy*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*velocityLength)/m+(C_wxy*(vx_res)*((q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vy_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vx_res)*2.0-(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(4,18) = -w;
A(4,19) = v;
A(4,29) = (C_wxy*velocityLength*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3))/m+(C_wxy*(vx_res)*1.0/velocityLength*((q_0*q_3*2.0-q_1*q_2*2.0)*(vy_res)*2.0-(q_0*q_2*2.0+q_1*q_3*2.0)*(vz_res)*2.0+(vx_res)*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0)*(1.0/2.0))/m;
A(4,30) = (C_wxy*(q_0*q_3*2.0+q_1*q_2*2.0)*velocityLength)/m+(C_wxy*(vx_res)*1.0/velocityLength*((q_0*q_3*2.0+q_1*q_2*2.0)*(vx_res)*2.0-(vy_res)*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+(q_0*q_1*2.0-q_2*q_3*2.0)*(vz_res)*2.0)*(1.0/2.0))/m;
A(4,31) = ((vx_res)*velocityLength)/m;
A(5,4) = -r+(C_wxy*(u*-2.0+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
A(5,5) = -(C_wxy*velocityLength)/m-(C_wxy*(v*2.0-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
A(5,6) = p-(C_wxy*(w*2.0-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
A(5,7) = g*q_1*2.0+(C_wxy*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*velocityLength)/m-(C_wxy*((q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vy_res)*-2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vx_res)*2.0+(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
A(5,8) = g*q_0*2.0-(C_wxy*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*velocityLength)/m-(C_wxy*((q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vy_res)*2.0+(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vx_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
A(5,9) = g*q_3*2.0+(C_wxy*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*velocityLength)/m+(C_wxy*((q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vy_res)*2.0-(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vx_res)*2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
A(5,10) = g*q_2*2.0-(C_wxy*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*velocityLength)/m-(C_wxy*((q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vy_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vx_res)*2.0-(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(vy_res)*(1.0/2.0))/m;
A(5,17) = w;
A(5,19) = -u;
A(5,29) = -(C_wxy*(q_0*q_3*2.0-q_1*q_2*2.0)*velocityLength)/m-(C_wxy*1.0/velocityLength*(vy_res)*((q_0*q_3*2.0-q_1*q_2*2.0)*(vy_res)*2.0-(q_0*q_2*2.0+q_1*q_3*2.0)*(vz_res)*2.0+(vx_res)*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0)*(1.0/2.0))/m;
A(5,30) = (C_wxy*velocityLength*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3))/m-(C_wxy*1.0/velocityLength*(vy_res)*((q_0*q_3*2.0+q_1*q_2*2.0)*(vx_res)*2.0-(vy_res)*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+(q_0*q_1*2.0-q_2*q_3*2.0)*(vz_res)*2.0)*(1.0/2.0))/m;
A(5,31) = -(velocityLength*(vy_res))/m;
A(6,4) = q+(C_wz*(u*-2.0+v_x_wind*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0+v_y_wind*(q_0*q_3*2.0+q_1*q_2*2.0)*2.0)*(vz_res)*1.0/velocityLength*(1.0/2.0))/m;
A(6,5) = -p-(C_wz*(v*2.0-v_y_wind*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+v_x_wind*(q_0*q_3*2.0-q_1*q_2*2.0)*2.0)*(vz_res)*1.0/velocityLength*(1.0/2.0))/m;
A(6,6) = (CT1s*w_1+CT1s*w_2+CT1s*w_3+CT1s*w_4+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*velocityLength)/m-(C_wz*(vz_res)*(w*2.0-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(6,7) = g*q_0*2.0+(CT1s*w_1*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT1s*w_2*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT1s*w_3*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT1s*w_4*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*velocityLength)/m-(C_wz*(vz_res)*((q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vy_res)*-2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vx_res)*2.0+(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(6,8) = g*q_1*-2.0+(CT1s*w_1*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT1s*w_2*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT1s*w_3*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT1s*w_4*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*velocityLength)/m-(C_wz*(vz_res)*((q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vy_res)*2.0+(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vx_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(6,9) = g*q_2*-2.0-(CT1s*w_1*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT1s*w_2*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT1s*w_3*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT1s*w_4*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m+(C_wz*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*velocityLength)/m+(C_wz*(vz_res)*((q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vy_res)*2.0-(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(vx_res)*2.0+(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(6,10) = g*q_3*2.0-(CT1s*w_1*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT1s*w_2*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT1s*w_3*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT1s*w_4*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m+(C_wz*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*velocityLength)/m-(C_wz*(vz_res)*((q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(vy_res)*2.0+(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(vx_res)*2.0-(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(vz_res)*2.0)*1.0/velocityLength*(1.0/2.0))/m;
A(6,17) = -v+(CT2s*l_m*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*l_m*w_2-CT1s*l_m*w_4-CT2s*l_m*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*l_m*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*l_m*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
A(6,18) = u+(CT2s*l_m*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT1s*l_m*w_1+CT1s*l_m*w_3-CT2s*l_m*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*l_m*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*l_m*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
A(6,20) = -(CT0s*w_1*2.0+CT1s*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0))/m;
A(6,21) = -(CT0s*w_2*2.0-CT1s*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
A(6,22) = -(CT0s*w_3*2.0-CT1s*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m;
A(6,23) = -(CT0s*w_4*2.0+CT1s*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0))/m;
A(6,25) = -(w_1*w_1+w_2*w_2+w_3*w_3+w_4*w_4)/m;
A(6,29) = -(CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_1*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_2*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_3*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_4*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m+(C_wz*(q_0*q_2*2.0+q_1*q_3*2.0)*velocityLength)/m-(C_wz*(vz_res)*1.0/velocityLength*((q_0*q_3*2.0-q_1*q_2*2.0)*(vy_res)*2.0-(q_0*q_2*2.0+q_1*q_3*2.0)*(vz_res)*2.0+(vx_res)*(q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3)*2.0)*(1.0/2.0))/m;
A(6,30) = (CT2s*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_1*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_2*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_3*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_4*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0))/m-(C_wz*(q_0*q_1*2.0-q_2*q_3*2.0)*velocityLength)/m-(C_wz*(vz_res)*1.0/velocityLength*((q_0*q_3*2.0+q_1*q_2*2.0)*(vx_res)*2.0-(vy_res)*(q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3)*2.0+(q_0*q_1*2.0-q_2*q_3*2.0)*(vz_res)*2.0)*(1.0/2.0))/m;
A(7,7) = (q_0*q_0)*-9.0-(q_1*q_1)*3.0-(q_2*q_2)*3.0-(q_3*q_3)*3.0+3.0;
A(7,8) = p*(-1.0/2.0)-q_0*q_1*6.0;
A(7,9) = q*(-1.0/2.0)-q_0*q_2*6.0;
A(7,10) = r*(-1.0/2.0)-q_0*q_3*6.0;
A(7,17) = q_1*(-1.0/2.0);
A(7,18) = q_2*(-1.0/2.0);
A(7,19) = q_3*(-1.0/2.0);
A(8,7) = p*(1.0/2.0)-q_0*q_1*6.0;
A(8,8) = (q_0*q_0)*-3.0-(q_1*q_1)*9.0-(q_2*q_2)*3.0-(q_3*q_3)*3.0+3.0;
A(8,9) = r*(1.0/2.0)-q_1*q_2*6.0;
A(8,10) = q*(-1.0/2.0)-q_1*q_3*6.0;
A(8,17) = q_0*(1.0/2.0);
A(8,18) = q_3*(-1.0/2.0);
A(8,19) = q_2*(1.0/2.0);
A(9,7) = q*(1.0/2.0)-q_0*q_2*6.0;
A(9,8) = r*(-1.0/2.0)-q_1*q_2*6.0;
A(9,9) = (q_0*q_0)*-3.0-(q_1*q_1)*3.0-(q_2*q_2)*9.0-(q_3*q_3)*3.0+3.0;
A(9,10) = p*(1.0/2.0)-q_2*q_3*6.0;
A(9,17) = q_3*(1.0/2.0);
A(9,18) = q_0*(1.0/2.0);
A(9,19) = q_1*(-1.0/2.0);
A(10,7) = r*(1.0/2.0)-q_0*q_3*6.0;
A(10,8) = q*(1.0/2.0)-q_1*q_3*6.0;
A(10,9) = p*(-1.0/2.0)-q_2*q_3*6.0;
A(10,10) = (q_0*q_0)*-3.0-(q_1*q_1)*3.0-(q_2*q_2)*3.0-(q_3*q_3)*9.0+3.0;
A(10,17) = q_2*(-1.0/2.0);
A(10,18) = q_1*(1.0/2.0);
A(10,19) = q_0*(1.0/2.0);
A(17,6) = -(-CT2s*l_m*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)-CT1s*l_m*w_2+CT1s*l_m*w_4+CT2s*l_m*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*(l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))-CT2s*l_m*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))-CT2s*l_m*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0)/I_x;
A(17,7) = -(-CT2s*(l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)-CT2s*l_m*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)-CT2s*l_m*q_1*v_y_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*v_x_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT1s*l_m*q_1*v_y_wind*w_2*2.0+CT1s*l_m*q_2*v_x_wind*w_2*2.0+CT1s*l_m*q_1*v_y_wind*w_4*2.0-CT1s*l_m*q_2*v_x_wind*w_4*2.0+CT2s*(l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*l_m*q_1*v_y_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*v_x_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0)/I_x;
A(17,8) = -(-CT2s*(l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)-CT2s*l_m*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)-CT2s*l_m*q_0*v_y_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_3*v_x_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT1s*l_m*q_0*v_y_wind*w_2*2.0+CT1s*l_m*q_0*v_y_wind*w_4*2.0+CT1s*l_m*q_3*v_x_wind*w_2*2.0-CT1s*l_m*q_3*v_x_wind*w_4*2.0+CT2s*(l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*l_m*q_0*v_y_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_3*v_x_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0)/I_x;
A(17,9) = (-CT2s*(l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)-CT2s*l_m*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)-CT2s*l_m*q_0*v_x_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_3*v_y_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT1s*l_m*q_0*v_x_wind*w_2*2.0+CT1s*l_m*q_0*v_x_wind*w_4*2.0-CT1s*l_m*q_3*v_y_wind*w_2*2.0+CT1s*l_m*q_3*v_y_wind*w_4*2.0+CT2s*(l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*l_m*q_0*v_x_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_3*v_y_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0)/I_x;
A(17,10) = (-CT2s*(l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)-CT2s*l_m*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)-CT2s*l_m*q_1*v_x_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*v_y_wind*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT1s*l_m*q_1*v_x_wind*w_2*2.0+CT1s*l_m*q_1*v_x_wind*w_4*2.0-CT1s*l_m*q_2*v_y_wind*w_2*2.0+CT1s*l_m*q_2*v_y_wind*w_4*2.0+CT2s*(l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*l_m*q_1*v_x_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*v_y_wind*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0)/I_x;
A(17,17) = (-C_mxy*sqrt(p*p+q*q+r*r)+CT2s*(l_m*l_m)*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-C_mxy*(p*p)*1.0/(sqrt(p*p+q*q+r*r)+eps)+CT2s*(l_m*l_m)*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*(l_m*l_m)*w_2+CT1s*(l_m*l_m)*w_4+CT2s*(l_m*l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m)*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))-CT2s*(l_m*l_m)*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m)*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*(l_m*l_m)*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*(l_m*l_m)*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*(l_m*l_m)*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0)/I_x;
A(17,18) = -(-I_y*r+I_z*r+C_mxy*p*q*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_x;
A(17,19) = -(-I_y*q+I_z*q+C_mxy*p*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_x;
A(17,21) = -(-CT1s*l_m*w+CT0s*l_m*w_2*2.0-CT1s*(l_m*l_m)*p-CT1s*l_m*q_0*q_1*v_y_wind*2.0+CT1s*l_m*q_0*q_2*v_x_wind*2.0+CT1s*l_m*q_1*q_3*v_x_wind*2.0+CT1s*l_m*q_2*q_3*v_y_wind*2.0)/I_x;
A(17,23) = (-CT1s*l_m*w+CT0s*l_m*w_4*2.0+CT1s*(l_m*l_m)*p-CT1s*l_m*q_0*q_1*v_y_wind*2.0+CT1s*l_m*q_0*q_2*v_x_wind*2.0+CT1s*l_m*q_1*q_3*v_x_wind*2.0+CT1s*l_m*q_2*q_3*v_y_wind*2.0)/I_x;
A(17,25) = -(l_m*(w_2*w_2)-l_m*(w_4*w_4))/I_x;
A(17,26) = 1.0/I_x;
A(17,29) = (-CT2s*(l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)-CT2s*l_m*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)-CT2s*l_m*q_0*q_2*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_1*q_3*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT1s*l_m*q_0*q_2*w_2*2.0+CT1s*l_m*q_0*q_2*w_4*2.0-CT1s*l_m*q_1*q_3*w_2*2.0+CT1s*l_m*q_1*q_3*w_4*2.0+CT2s*(l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*l_m*q_0*q_2*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0)/I_x;
A(17,30) = -(-CT2s*(l_m*l_m)*p*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)-CT2s*l_m*w*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)-CT2s*l_m*q_0*q_1*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*abs(w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT1s*l_m*q_0*q_1*w_2*2.0+CT1s*l_m*q_0*q_1*w_4*2.0+CT1s*l_m*q_2*q_3*w_2*2.0-CT1s*l_m*q_2*q_3*w_4*2.0+CT2s*(l_m*l_m)*p*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*l_m*q_0*q_1*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*q_3*abs(-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*p-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*p+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)/I_x;
A(18,6) = -(-CT2s*l_m*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*l_m*w_1-CT1s*l_m*w_3+CT2s*l_m*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-CT2s*(l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))-CT2s*l_m*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))-CT2s*l_m*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0)/I_y;
A(18,7) = -(-CT2s*(l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)-CT2s*l_m*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)-CT2s*l_m*q_1*v_y_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*v_x_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT1s*l_m*q_1*v_y_wind*w_1*2.0-CT1s*l_m*q_2*v_x_wind*w_1*2.0-CT1s*l_m*q_1*v_y_wind*w_3*2.0+CT1s*l_m*q_2*v_x_wind*w_3*2.0+CT2s*(l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*l_m*q_1*v_y_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*v_x_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*2.0)/I_y;
A(18,8) = -(-CT2s*(l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)-CT2s*l_m*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)-CT2s*l_m*q_0*v_y_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_3*v_x_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT1s*l_m*q_0*v_y_wind*w_1*2.0-CT1s*l_m*q_0*v_y_wind*w_3*2.0-CT1s*l_m*q_3*v_x_wind*w_1*2.0+CT1s*l_m*q_3*v_x_wind*w_3*2.0+CT2s*(l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*l_m*q_0*v_y_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_3*v_x_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*2.0)/I_y;
A(18,9) = (-CT2s*(l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)-CT2s*l_m*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)-CT2s*l_m*q_0*v_x_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_3*v_y_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT1s*l_m*q_0*v_x_wind*w_1*2.0-CT1s*l_m*q_0*v_x_wind*w_3*2.0+CT1s*l_m*q_3*v_y_wind*w_1*2.0-CT1s*l_m*q_3*v_y_wind*w_3*2.0+CT2s*(l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*l_m*q_0*v_x_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_3*v_y_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*2.0)/I_y;
A(18,10) = (-CT2s*(l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)-CT2s*l_m*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)-CT2s*l_m*q_1*v_x_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*v_y_wind*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT1s*l_m*q_1*v_x_wind*w_1*2.0-CT1s*l_m*q_1*v_x_wind*w_3*2.0+CT1s*l_m*q_2*v_y_wind*w_1*2.0-CT1s*l_m*q_2*v_y_wind*w_3*2.0+CT2s*(l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*l_m*q_1*v_x_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*v_y_wind*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*2.0)/I_y;
A(18,17) = -(I_x*r-I_z*r+C_mxy*p*q*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_y;
A(18,18) = (-C_mxy*sqrt(p*p+q*q+r*r)+CT2s*(l_m*l_m)*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)-C_mxy*(q*q)*1.0/(sqrt(p*p+q*q+r*r)+eps)+CT2s*(l_m*l_m)*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)+CT1s*(l_m*l_m)*w_1+CT1s*(l_m*l_m)*w_3+CT2s*(l_m*l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m)*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))-CT2s*(l_m*l_m)*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))+CT2s*(l_m*l_m)*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*2.0-CT2s*(l_m*l_m)*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*(l_m*l_m)*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*(l_m*l_m)*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0+CT2s*(l_m*l_m)*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*2.0)/I_y;
A(18,19) = -(I_x*p-I_z*p+C_mxy*q*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_y;
A(18,20) = (-CT1s*l_m*w+CT0s*l_m*w_1*2.0+CT1s*(l_m*l_m)*q-CT1s*l_m*q_0*q_1*v_y_wind*2.0+CT1s*l_m*q_0*q_2*v_x_wind*2.0+CT1s*l_m*q_1*q_3*v_x_wind*2.0+CT1s*l_m*q_2*q_3*v_y_wind*2.0)/I_y;
A(18,22) = -(-CT1s*l_m*w+CT0s*l_m*w_3*2.0-CT1s*(l_m*l_m)*q-CT1s*l_m*q_0*q_1*v_y_wind*2.0+CT1s*l_m*q_0*q_2*v_x_wind*2.0+CT1s*l_m*q_1*q_3*v_x_wind*2.0+CT1s*l_m*q_2*q_3*v_y_wind*2.0)/I_y;
A(18,25) = (l_m*(w_1*w_1)-l_m*(w_3*w_3))/I_y;
A(18,27) = 1.0/I_y;
A(18,29) = (-CT2s*(l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)-CT2s*l_m*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)-CT2s*l_m*q_0*q_2*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_1*q_3*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT1s*l_m*q_0*q_2*w_1*2.0-CT1s*l_m*q_0*q_2*w_3*2.0+CT1s*l_m*q_1*q_3*w_1*2.0-CT1s*l_m*q_1*q_3*w_3*2.0+CT2s*(l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*l_m*q_0*q_2*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_1*q_3*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_2*2.0+q_1*q_3*2.0)*2.0)/I_y;
A(18,30) = -(-CT2s*(l_m*l_m)*q*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)-CT2s*l_m*w*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)-CT2s*l_m*q_0*q_1*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT2s*l_m*q_2*q_3*abs(w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)*2.0+CT1s*l_m*q_0*q_1*w_1*2.0-CT1s*l_m*q_0*q_1*w_3*2.0-CT1s*l_m*q_2*q_3*w_1*2.0+CT1s*l_m*q_2*q_3*w_3*2.0+CT2s*(l_m*l_m)*q*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*l_m*q_0*q_1*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*q_2*q_3*abs(-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)*2.0-CT2s*l_m*w*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)-CT2s*l_m*q_0*q_1*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((-w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)/abs(eps + -w+l_m*q-q_0*q_1*v_y_wind*2.0+q_0*q_2*v_x_wind*2.0+q_1*q_3*v_x_wind*2.0+q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0-CT2s*l_m*q_0*q_1*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_0*q_2*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_1*q_3*v_x_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0+CT2s*l_m*q_2*q_3*v_y_wind*(((w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)/abs(eps + w+l_m*q+q_0*q_1*v_y_wind*2.0-q_0*q_2*v_x_wind*2.0-q_1*q_3*v_x_wind*2.0-q_2*q_3*v_y_wind*2.0)))*(q_0*q_1*2.0-q_2*q_3*2.0)*2.0)/I_y;
A(19,17) = (I_x*R_A*q-I_y*R_A*q-C_mz*R_A*p*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/(I_z*R_A);
A(19,18) = (I_x*R_A*p-I_y*R_A*p-C_mz*R_A*q*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/(I_z*R_A);
A(19,19) = (-C_mz*R_A*sqrt(p*p+q*q+r*r)-C_mz*R_A*(r*r)*1.0/(sqrt(p*p+q*q+r*r)+eps))/(I_z*R_A);
A(19,20) = ((Psi*Psi)*(1.0/2.0)-Psi*((R_A/abs(eps + R_A)))*((Psi*Psi)*w_1*2.0-Psi*U_1*beta_m*2.0)*1.0/sqrt((U_1*U_1)*(beta_m*beta_m)+(Psi*Psi)*(w_1*w_1)+R_A*U_1*alpha_m*4.0-Psi*U_1*beta_m*w_1*2.0)*(1.0/4.0))/(I_z*R_A);
A(19,21) = ((Psi*Psi)*(-1.0/2.0)+Psi*((R_A/abs(eps + R_A)))*((Psi*Psi)*w_2*2.0-Psi*U_2*beta_m*2.0)*1.0/sqrt((U_2*U_2)*(beta_m*beta_m)+(Psi*Psi)*(w_2*w_2)+R_A*U_2*alpha_m*4.0-Psi*U_2*beta_m*w_2*2.0)*(1.0/4.0))/(I_z*R_A);
A(19,22) = ((Psi*Psi)*(1.0/2.0)-Psi*((R_A/abs(eps + R_A)))*((Psi*Psi)*w_3*2.0-Psi*U_3*beta_m*2.0)*1.0/sqrt((U_3*U_3)*(beta_m*beta_m)+(Psi*Psi)*(w_3*w_3)+R_A*U_3*alpha_m*4.0-Psi*U_3*beta_m*w_3*2.0)*(1.0/4.0))/(I_z*R_A);
A(19,23) = ((Psi*Psi)*(-1.0/2.0)+Psi*((R_A/abs(eps + R_A)))*((Psi*Psi)*w_4*2.0-Psi*U_4*beta_m*2.0)*1.0/sqrt((U_4*U_4)*(beta_m*beta_m)+(Psi*Psi)*(w_4*w_4)+R_A*U_4*alpha_m*4.0-Psi*U_4*beta_m*w_4*2.0)*(1.0/4.0))/(I_z*R_A);
A(19,28) = 1.0/I_z;
A(20,6) = (k_t*(CT1s*w_1+CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,7) = (k_t*(CT1s*w_1*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,8) = (k_t*(CT1s*w_1*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,9) = -(k_t*(CT1s*w_1*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,10) = -(k_t*(CT1s*w_1*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,18) = -(k_t*(CT1s*l_m*w_1+CT2s*l_m*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*l_m*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,20) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(1.0/(R_A*R_A)*power(U_1*beta_m-Psi*w_1,2.0)*(1.0/4.0)+(U_1*alpha_m)/R_A)*(U_1*beta_m-Psi*w_1)*(1.0/4.0))+k_t*(CT0s*w_1*2.0-CT1s*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,24) = (-CT0s*(w_1*w_1)+CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_1*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)))/J_M;
A(20,25) = -(k_t*(w_1*w_1))/J_M;
A(20,29) = -(k_t*(CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_1*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(20,30) = (k_t*(CT2s*abs(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_1*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*(((w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w-l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,6) = (k_t*(CT1s*w_2+CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,7) = (k_t*(CT1s*w_2*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,8) = (k_t*(CT1s*w_2*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,9) = -(k_t*(CT1s*w_2*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,10) = -(k_t*(CT1s*w_2*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,17) = (k_t*(CT1s*l_m*w_2+CT2s*l_m*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*l_m*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,21) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(1.0/(R_A*R_A)*power(U_2*beta_m-Psi*w_2,2.0)*(1.0/4.0)+(U_2*alpha_m)/R_A)*(U_2*beta_m-Psi*w_2)*(1.0/4.0))+k_t*(CT0s*w_2*2.0-CT1s*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,24) = (-CT0s*(w_2*w_2)+CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_2*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)))/J_M;
A(21,25) = -(k_t*(w_2*w_2))/J_M;
A(21,29) = -(k_t*(CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_2*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(21,30) = (k_t*(CT2s*abs(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_2*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*(((w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w+l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,6) = (k_t*(CT1s*w_3+CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,7) = (k_t*(CT1s*w_3*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,8) = (k_t*(CT1s*w_3*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,9) = -(k_t*(CT1s*w_3*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,10) = -(k_t*(CT1s*w_3*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,18) = (k_t*(CT1s*l_m*w_3+CT2s*l_m*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*l_m*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,22) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(1.0/(R_A*R_A)*power(U_3*beta_m-Psi*w_3,2.0)*(1.0/4.0)+(U_3*alpha_m)/R_A)*(U_3*beta_m-Psi*w_3)*(1.0/4.0))+k_t*(CT0s*w_3*2.0-CT1s*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,24) = (-CT0s*(w_3*w_3)+CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_3*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)))/J_M;
A(22,25) = -(k_t*(w_3*w_3))/J_M;
A(22,29) = -(k_t*(CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_3*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(22,30) = (k_t*(CT2s*abs(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_3*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*(((w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w+l_m*q-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,6) = (k_t*(CT1s*w_4+CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,7) = (k_t*(CT1s*w_4*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)+CT2s*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_y_wind*2.0-q_2*v_x_wind*2.0)*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,8) = (k_t*(CT1s*w_4*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)+CT2s*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_y_wind*2.0-q_3*v_x_wind*2.0)*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,9) = -(k_t*(CT1s*w_4*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)+CT2s*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*v_x_wind*2.0+q_3*v_y_wind*2.0)*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,10) = -(k_t*(CT1s*w_4*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)+CT2s*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_1*v_x_wind*2.0+q_2*v_y_wind*2.0)*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,17) = -(k_t*(CT1s*l_m*w_4+CT2s*l_m*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT2s*l_m*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,23) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(1.0/(R_A*R_A)*power(U_4*beta_m-Psi*w_4,2.0)*(1.0/4.0)+(U_4*alpha_m)/R_A)*(U_4*beta_m-Psi*w_4)*(1.0/4.0))+k_t*(CT0s*w_4*2.0-CT1s*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,24) = (-CT0s*(w_4*w_4)+CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))+CT1s*w_4*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0)))/J_M;
A(23,25) = -(k_t*(w_4*w_4))/J_M;
A(23,29) = -(k_t*(CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_2*2.0+q_1*q_3*2.0)+CT1s*w_4*(q_0*q_2*2.0+q_1*q_3*2.0)+CT2s*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_2*2.0+q_1*q_3*2.0)*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;
A(23,30) = (k_t*(CT2s*abs(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))*(q_0*q_1*2.0-q_2*q_3*2.0)+CT1s*w_4*(q_0*q_1*2.0-q_2*q_3*2.0)+CT2s*(((w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))/abs(eps + w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))*(q_0*q_1*2.0-q_2*q_3*2.0)*(w-l_m*p-v_x_wind*(q_0*q_2*2.0+q_1*q_3*2.0)+v_y_wind*(q_0*q_1*2.0-q_2*q_3*2.0))))/J_M;

  W(1,1) = -q_1;
  W(1,2) = q_0;
  W(1,3) = q_3;
  W(1,4) = -q_2;
  W(2,1) = -q_2;
  W(2,2) = -q_3;
  W(2,3) = q_0;
  W(2,4) = q_1;
  W(3,1) = -q_3;
  W(3,2) = q_2;
  W(3,3) = -q_1;
  W(3,4) = q_0;

B = zeros(31,4);
  
B(19,1) = (Psi*beta_m*(-1.0/2.0)-Psi*((R_A/abs(R_A)))*(R_A*alpha_m*4.0+U_1*(beta_m*beta_m)*2.0-Psi*beta_m*w_1*2.0)*1.0/sqrt(eps + ( eps + U_1*U_1)*(beta_m*beta_m)+(Psi*Psi)*(w_1*w_1)+R_A*U_1*alpha_m*4.0-Psi*U_1*beta_m*w_1*2.0)*(1.0/4.0))/(I_z*R_A);
B(19,2) = (Psi*beta_m*(1.0/2.0)+Psi*((R_A/abs(R_A)))*(R_A*alpha_m*4.0+U_2*(beta_m*beta_m)*2.0-Psi*beta_m*w_2*2.0)*1.0/sqrt(eps + ( eps + U_2*U_2)*(beta_m*beta_m)+(Psi*Psi)*(w_2*w_2)+R_A*U_2*alpha_m*4.0-Psi*U_2*beta_m*w_2*2.0)*(1.0/4.0))/(I_z*R_A);
B(19,3) = (Psi*beta_m*(-1.0/2.0)-Psi*((R_A/abs(R_A)))*(R_A*alpha_m*4.0+U_3*(beta_m*beta_m)*2.0-Psi*beta_m*w_3*2.0)*1.0/sqrt(eps + ( eps + U_3*U_3)*(beta_m*beta_m)+(Psi*Psi)*(w_3*w_3)+R_A*U_3*alpha_m*4.0-Psi*U_3*beta_m*w_3*2.0)*(1.0/4.0))/(I_z*R_A);
B(19,4) = (Psi*beta_m*(1.0/2.0)+Psi*((R_A/abs(R_A)))*(R_A*alpha_m*4.0+U_4*(beta_m*beta_m)*2.0-Psi*beta_m*w_4*2.0)*1.0/sqrt(eps + ( eps + U_4*U_4)*(beta_m*beta_m)+(Psi*Psi)*(w_4*w_4)+R_A*U_4*alpha_m*4.0-Psi*U_4*beta_m*w_4*2.0)*(1.0/4.0))/(I_z*R_A);
  
B(20,1) = (Psi*((beta_m*(1.0/2.0))/R_A+1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_1*beta_m-Psi*w_1,2.0)*(1.0/4.0)+(U_1*alpha_m)/R_A)*(alpha_m/R_A+1.0/(R_A*R_A)*beta_m*(U_1*beta_m-Psi*w_1)*(1.0/2.0))*(1.0/2.0)))/J_M;
B(21,2) = (Psi*((beta_m*(1.0/2.0))/R_A+1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_2*beta_m-Psi*w_2,2.0)*(1.0/4.0)+(U_2*alpha_m)/R_A)*(alpha_m/R_A+1.0/(R_A*R_A)*beta_m*(U_2*beta_m-Psi*w_2)*(1.0/2.0))*(1.0/2.0)))/J_M;
B(22,3) = (Psi*((beta_m*(1.0/2.0))/R_A+1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_3*beta_m-Psi*w_3,2.0)*(1.0/4.0)+(U_3*alpha_m)/R_A)*(alpha_m/R_A+1.0/(R_A*R_A)*beta_m*(U_3*beta_m-Psi*w_3)*(1.0/2.0))*(1.0/2.0)))/J_M;
B(23,4) = (Psi*((beta_m*(1.0/2.0))/R_A+1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_4*beta_m-Psi*w_4,2.0)*(1.0/4.0)+(U_4*alpha_m)/R_A)*(alpha_m/R_A+1.0/(R_A*R_A)*beta_m*(U_4*beta_m-Psi*w_4)*(1.0/2.0))*(1.0/2.0)))/J_M;

Q(1:6,1:6)   = diag(Qin(1:6));
Q(7:10,7:10) = 1/4*transpose(-W)*diag(Qin(7:9))*(-W);
Q(11:end,11:end) = diag(Qin(10:end));

Q = Q + B*(diag(Qin(19:22)))*B';

A = eye(size(A)) + dt*A + 1/2*(A*dt)^2;

% 
% A   = zeros(28);
% Q   = zeros(28);
% W   = zeros(3,4);
% 
% A(1,4) = q_0*q_0+q_1*q_1-q_2*q_2-q_3*q_3;
% A(1,5) = q_0*q_3*-2.0+q_1*q_2*2.0;
% A(1,6) = q_0*q_2*2.0+q_1*q_3*2.0;
% A(1,7) = q_0*u*2.0-q_3*v*2.0+q_2*w*2.0;
% A(1,8) = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
% A(1,9) = q_2*u*-2.0+q_1*v*2.0+q_0*w*2.0;
% A(1,10) = q_3*u*-2.0-q_0*v*2.0+q_1*w*2.0;
% A(2,4) = q_0*q_3*2.0+q_1*q_2*2.0;
% A(2,5) = q_0*q_0-q_1*q_1+q_2*q_2-q_3*q_3;
% A(2,6) = q_0*q_1*-2.0+q_2*q_3*2.0;
% A(2,7) = q_3*u*2.0+q_0*v*2.0-q_1*w*2.0;
% A(2,8) = q_2*u*2.0-q_1*v*2.0-q_0*w*2.0;
% A(2,9) = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
% A(2,10) = q_0*u*2.0-q_3*v*2.0+q_2*w*2.0;
% A(3,4) = q_0*q_2*-2.0+q_1*q_3*2.0;
% A(3,5) = q_0*q_1*2.0+q_2*q_3*2.0;
% A(3,6) = q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3;
% A(3,7) = q_2*u*-2.0+q_1*v*2.0+q_0*w*2.0;
% A(3,8) = q_3*u*2.0+q_0*v*2.0-q_1*w*2.0;
% A(3,9) = q_0*u*-2.0+q_3*v*2.0-q_2*w*2.0;
% A(3,10) = q_1*u*2.0+q_2*v*2.0+q_3*w*2.0;
% A(4,4) = -(C_wxy*sqrt(u*u+v*v+w*w))/m-(C_wxy*(u*u)*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(4,5) = r-(C_wxy*u*v*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(4,6) = -q-(C_wxy*u*w*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(4,7) = g*q_2*-2.0;
% A(4,8) = g*q_3*2.0;
% A(4,9) = g*q_0*-2.0;
% A(4,10) = g*q_1*2.0;
% A(4,18) = -w;
% A(4,19) = v;
% A(5,4) = -r-(C_wxy*u*v*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(5,5) = -(C_wxy*sqrt(u*u+v*v+w*w))/m-(C_wxy*(v*v)*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(5,6) = p-(C_wxy*v*w*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(5,7) = g*q_1*2.0;
% A(5,8) = g*q_0*2.0;
% A(5,9) = g*q_3*2.0;
% A(5,10) = g*q_2*2.0;
% A(5,17) = w;
% A(5,19) = -u;
% A(6,4) = q-(C_wz*u*w*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(6,5) = -p-(C_wz*v*w*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(6,6) = (CT2s*abs(w+l_m*p)+CT2s*abs(w-l_m*p)+CT2s*abs(w+l_m*q)+CT2s*abs(w-l_m*q)+CT1s*w_1+CT1s*w_2+CT1s*w_3+CT1s*w_4+CT2s*(((w+l_m*p)/abs(eps + w+l_m*p)))*(w+l_m*p)+CT2s*(((w-l_m*p)/abs(eps + w-l_m*p)))*(w-l_m*p)+CT2s*(((w+l_m*q)/abs(eps + w+l_m*q)))*(w+l_m*q)+CT2s*(((w-l_m*q)/abs(eps + w-l_m*q)))*(w-l_m*q))/m-(C_wz*sqrt(u*u+v*v+w*w))/m-(C_wz*(w*w)*1.0/(sqrt(u*u+v*v+w*w)+eps))/m;
% A(6,7) = g*q_0*2.0;
% A(6,8) = g*q_1*-2.0;
% A(6,9) = g*q_2*-2.0;
% A(6,10) = g*q_3*2.0;
% A(6,17) = -v+(CT2s*l_m*abs(w+l_m*p)-CT2s*l_m*abs(w-l_m*p)+CT1s*l_m*w_2-CT1s*l_m*w_4+CT2s*l_m*(((w+l_m*p)/abs(eps + w+l_m*p)))*(w+l_m*p)-CT2s*l_m*(((w-l_m*p)/abs(eps + w-l_m*p)))*(w-l_m*p))/m;
% A(6,18) = u+(CT2s*l_m*abs(w+l_m*q)-CT2s*l_m*abs(w-l_m*q)-CT1s*l_m*w_1+CT1s*l_m*w_3+CT2s*l_m*(((w+l_m*q)/abs(eps + w+l_m*q)))*(w+l_m*q)-CT2s*l_m*(((w-l_m*q)/abs(eps + w-l_m*q)))*(w-l_m*q))/m;
% A(6,20) = -(CT0s*w_1*2.0-CT1s*(w-l_m*q))/m;
% A(6,21) = -(CT0s*w_2*2.0-CT1s*(w+l_m*p))/m;
% A(6,22) = -(CT0s*w_3*2.0-CT1s*(w+l_m*q))/m;
% A(6,23) = -(CT0s*w_4*2.0-CT1s*(w-l_m*p))/m;
% A(6,25) = -(w_1*w_1+w_2*w_2+w_3*w_3+w_4*w_4)/m;
% A(7,7) = (q_0*q_0)*-9.0-(q_1*q_1)*3.0-(q_2*q_2)*3.0-(q_3*q_3)*3.0+3.0;
% A(7,8) = p*(-1.0/2.0)-q_0*q_1*6.0;
% A(7,9) = q*(-1.0/2.0)-q_0*q_2*6.0;
% A(7,10) = r*(-1.0/2.0)-q_0*q_3*6.0;
% A(7,17) = q_1*(-1.0/2.0);
% A(7,18) = q_2*(-1.0/2.0);
% A(7,19) = q_3*(-1.0/2.0);
% A(8,7) = p*(1.0/2.0)-q_0*q_1*6.0;
% A(8,8) = (q_0*q_0)*-3.0-(q_1*q_1)*9.0-(q_2*q_2)*3.0-(q_3*q_3)*3.0+3.0;
% A(8,9) = r*(1.0/2.0)-q_1*q_2*6.0;
% A(8,10) = q*(-1.0/2.0)-q_1*q_3*6.0;
% A(8,17) = q_0*(1.0/2.0);
% A(8,18) = q_3*(-1.0/2.0);
% A(8,19) = q_2*(1.0/2.0);
% A(9,7) = q*(1.0/2.0)-q_0*q_2*6.0;
% A(9,8) = r*(-1.0/2.0)-q_1*q_2*6.0;
% A(9,9) = (q_0*q_0)*-3.0-(q_1*q_1)*3.0-(q_2*q_2)*9.0-(q_3*q_3)*3.0+3.0;
% A(9,10) = p*(1.0/2.0)-q_2*q_3*6.0;
% A(9,17) = q_3*(1.0/2.0);
% A(9,18) = q_0*(1.0/2.0);
% A(9,19) = q_1*(-1.0/2.0);
% A(10,7) = r*(1.0/2.0)-q_0*q_3*6.0;
% A(10,8) = q*(1.0/2.0)-q_1*q_3*6.0;
% A(10,9) = p*(-1.0/2.0)-q_2*q_3*6.0;
% A(10,10) = (q_0*q_0)*-3.0-(q_1*q_1)*3.0-(q_2*q_2)*3.0-(q_3*q_3)*9.0+3.0;
% A(10,17) = q_2*(-1.0/2.0);
% A(10,18) = q_1*(1.0/2.0);
% A(10,19) = q_0*(1.0/2.0);
% A(17,6) = (CT2s*l_m*abs(w+l_m*p)-CT2s*l_m*abs(w-l_m*p)+CT1s*l_m*w_2-CT1s*l_m*w_4+CT2s*l_m*w*(((w+l_m*p)/abs(eps + w+l_m*p)))-CT2s*l_m*w*(((w-l_m*p)/abs(eps + w-l_m*p)))+CT2s*(l_m*l_m)*p*(((w+l_m*p)/abs(eps + w+l_m*p)))+CT2s*(l_m*l_m)*p*(((w-l_m*p)/abs(eps + w-l_m*p))))/I_x;
% A(17,17) = (-C_mxy*sqrt(p*p+q*q+r*r)-C_mxy*(p*p)*1.0/(sqrt(p*p+q*q+r*r)+eps)+CT2s*(l_m*l_m)*abs(w+l_m*p)+CT2s*(l_m*l_m)*abs(w-l_m*p)+CT1s*(l_m*l_m)*w_2+CT1s*(l_m*l_m)*w_4+CT2s*(l_m*l_m*l_m)*p*(((w+l_m*p)/abs(eps + w+l_m*p)))-CT2s*(l_m*l_m*l_m)*p*(((w-l_m*p)/abs(eps + w-l_m*p)))+CT2s*(l_m*l_m)*w*(((w+l_m*p)/abs(eps + w+l_m*p)))+CT2s*(l_m*l_m)*w*(((w-l_m*p)/abs(eps + w-l_m*p))))/I_x;
% A(17,18) = -(-I_y*r+I_z*r+C_mxy*p*q*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_x;
% A(17,19) = -(-I_y*q+I_z*q+C_mxy*p*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_x;
% A(17,21) = (CT1s*l_m*w-CT0s*l_m*w_2*2.0+CT1s*(l_m*l_m)*p)/I_x;
% A(17,23) = (-CT1s*l_m*w+CT0s*l_m*w_4*2.0+CT1s*(l_m*l_m)*p)/I_x;
% A(17,25) = -(l_m*(w_2*w_2)-l_m*(w_4*w_4))/I_x;
% A(17,26) = 1.0/I_x;
% A(18,6) = (CT2s*l_m*abs(w+l_m*q)-CT2s*l_m*abs(w-l_m*q)-CT1s*l_m*w_1+CT1s*l_m*w_3+CT2s*l_m*w*(((w+l_m*q)/abs(eps + w+l_m*q)))-CT2s*l_m*w*(((w-l_m*q)/abs(eps + w-l_m*q)))+CT2s*(l_m*l_m)*q*(((w+l_m*q)/abs(eps + w+l_m*q)))+CT2s*(l_m*l_m)*q*(((w-l_m*q)/abs(eps + w-l_m*q))))/I_y;
% A(18,17) = -(I_x*r-I_z*r+C_mxy*p*q*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_y;
% A(18,18) = (-C_mxy*sqrt(p*p+q*q+r*r)-C_mxy*(q*q)*1.0/(sqrt(p*p+q*q+r*r)+eps)+CT2s*(l_m*l_m)*abs(w+l_m*q)+CT2s*(l_m*l_m)*abs(w-l_m*q)+CT1s*(l_m*l_m)*w_1+CT1s*(l_m*l_m)*w_3+CT2s*(l_m*l_m*l_m)*q*(((w+l_m*q)/abs(eps + w+l_m*q)))-CT2s*(l_m*l_m*l_m)*q*(((w-l_m*q)/abs(eps + w-l_m*q)))+CT2s*(l_m*l_m)*w*(((w+l_m*q)/abs(eps + w+l_m*q)))+CT2s*(l_m*l_m)*w*(((w-l_m*q)/abs(eps + w-l_m*q))))/I_y;
% A(18,19) = -(I_x*p-I_z*p+C_mxy*q*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/I_y;
% A(18,20) = (-CT1s*l_m*w+CT0s*l_m*w_1*2.0+CT1s*(l_m*l_m)*q)/I_y;
% A(18,22) = (CT1s*l_m*w-CT0s*l_m*w_3*2.0+CT1s*(l_m*l_m)*q)/I_y;
% A(18,25) = (l_m*(w_1*w_1)-l_m*(w_3*w_3))/I_y;
% A(18,27) = 1.0/I_y;
% A(19,17) = (I_x*R_A*q-I_y*R_A*q-C_mz*R_A*p*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/(I_z*R_A);
% A(19,18) = (I_x*R_A*p-I_y*R_A*p-C_mz*R_A*q*r*1.0/(sqrt(p*p+q*q+r*r)+eps))/(I_z*R_A);
% A(19,19) = (-C_mz*R_A*sqrt(p*p+q*q+r*r)-C_mz*R_A*(r*r)*1.0/(sqrt(p*p+q*q+r*r)+eps))/(I_z*R_A);
% A(19,20) = ((Psi*Psi)*(1.0/2.0)-Psi*((Psi*Psi)*w_1*2.0-Psi*U_1*beta_m*2.0)*1.0/sqrt(eps + ( eps + U_1*U_1)*(beta_m*beta_m)+(Psi*Psi)*(w_1*w_1)+R_A*U_1*alpha_m*4.0-Psi*U_1*beta_m*w_1*2.0)*(1.0/4.0))/(I_z*R_A);
% A(19,21) = ((Psi*Psi)*(-1.0/2.0)+Psi*((Psi*Psi)*w_2*2.0-Psi*U_2*beta_m*2.0)*1.0/sqrt(eps + ( eps + U_2*U_2)*(beta_m*beta_m)+(Psi*Psi)*(w_2*w_2)+R_A*U_2*alpha_m*4.0-Psi*U_2*beta_m*w_2*2.0)*(1.0/4.0))/(I_z*R_A);
% A(19,22) = ((Psi*Psi)*(1.0/2.0)-Psi*((Psi*Psi)*w_3*2.0-Psi*U_3*beta_m*2.0)*1.0/sqrt(eps + ( eps + U_3*U_3)*(beta_m*beta_m)+(Psi*Psi)*(w_3*w_3)+R_A*U_3*alpha_m*4.0-Psi*U_3*beta_m*w_3*2.0)*(1.0/4.0))/(I_z*R_A);
% A(19,23) = ((Psi*Psi)*(-1.0/2.0)+Psi*((Psi*Psi)*w_4*2.0-Psi*U_4*beta_m*2.0)*1.0/sqrt(eps + ( eps + U_4*U_4)*(beta_m*beta_m)+(Psi*Psi)*(w_4*w_4)+R_A*U_4*alpha_m*4.0-Psi*U_4*beta_m*w_4*2.0)*(1.0/4.0))/(I_z*R_A);
% A(19,28) = 1.0/I_z;
% A(20,6) = (k_t*(CT2s*abs(w-l_m*q)+CT1s*w_1+CT2s*(((w-l_m*q)/abs(eps + w-l_m*q)))*(w-l_m*q)))/J_M;
% A(20,18) = -(k_t*(CT2s*l_m*abs(w-l_m*q)+CT1s*l_m*w_1+CT2s*l_m*(((w-l_m*q)/abs(eps + w-l_m*q)))*(w-l_m*q)))/J_M;
% A(20,20) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_1*beta_m-Psi*w_1,2.0)*(1.0/4.0)+(U_1*alpha_m)/R_A)*(U_1*beta_m-Psi*w_1)*(1.0/4.0))+k_t*(CT0s*w_1*2.0-CT1s*(w-l_m*q)))/J_M;
% A(20,24) = (-CT0s*(w_1*w_1)+CT2s*abs(w-l_m*q)*(w-l_m*q)+CT1s*w_1*(w-l_m*q))/J_M;
% A(20,25) = -(k_t*(w_1*w_1))/J_M;
% A(21,6) = (k_t*(CT2s*abs(w+l_m*p)+CT1s*w_2+CT2s*(((w+l_m*p)/abs(eps + w+l_m*p)))*(w+l_m*p)))/J_M;
% A(21,17) = (k_t*(CT2s*l_m*abs(w+l_m*p)+CT1s*l_m*w_2+CT2s*l_m*(((w+l_m*p)/abs(eps + w+l_m*p)))*(w+l_m*p)))/J_M;
% A(21,21) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_2*beta_m-Psi*w_2,2.0)*(1.0/4.0)+(U_2*alpha_m)/R_A)*(U_2*beta_m-Psi*w_2)*(1.0/4.0))+k_t*(CT0s*w_2*2.0-CT1s*(w+l_m*p)))/J_M;
% A(21,24) = (-CT0s*(w_2*w_2)+CT2s*abs(w+l_m*p)*(w+l_m*p)+CT1s*w_2*(w+l_m*p))/J_M;
% A(21,25) = -(k_t*(w_2*w_2))/J_M;
% A(22,6) = (k_t*(CT2s*abs(w+l_m*q)+CT1s*w_3+CT2s*(((w+l_m*q)/abs(eps + w+l_m*q)))*(w+l_m*q)))/J_M;
% A(22,18) = (k_t*(CT2s*l_m*abs(w+l_m*q)+CT1s*l_m*w_3+CT2s*l_m*(((w+l_m*q)/abs(eps + w+l_m*q)))*(w+l_m*q)))/J_M;
% A(22,22) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_3*beta_m-Psi*w_3,2.0)*(1.0/4.0)+(U_3*alpha_m)/R_A)*(U_3*beta_m-Psi*w_3)*(1.0/4.0))+k_t*(CT0s*w_3*2.0-CT1s*(w+l_m*q)))/J_M;
% A(22,24) = (-CT0s*(w_3*w_3)+CT2s*abs(w+l_m*q)*(w+l_m*q)+CT1s*w_3*(w+l_m*q))/J_M;
% A(22,25) = -(k_t*(w_3*w_3))/J_M;
% A(23,6) = (k_t*(CT2s*abs(w-l_m*p)+CT1s*w_4+CT2s*(((w-l_m*p)/abs(eps + w-l_m*p)))*(w-l_m*p)))/J_M;
% A(23,17) = -(k_t*(CT2s*l_m*abs(w-l_m*p)+CT1s*l_m*w_4+CT2s*l_m*(((w-l_m*p)/abs(eps + w-l_m*p)))*(w-l_m*p)))/J_M;
% A(23,23) = -(k_m+Psi*((Psi*(1.0/2.0))/R_A+Psi*1.0/(R_A*R_A)*1.0/sqrt(eps + 1.0/(R_A*R_A)*power(U_4*beta_m-Psi*w_4,2.0)*(1.0/4.0)+(U_4*alpha_m)/R_A)*(U_4*beta_m-Psi*w_4)*(1.0/4.0))+k_t*(CT0s*w_4*2.0-CT1s*(w-l_m*p)))/J_M;
% A(23,24) = (-CT0s*(w_4*w_4)+CT2s*abs(w-l_m*p)*(w-l_m*p)+CT1s*w_4*(w-l_m*p))/J_M;
% A(23,25) = -(k_t*(w_4*w_4))/J_M;
% 
%   W(1,1) = -q_1;
%   W(1,2) = q_0;
%   W(1,3) = q_3;
%   W(1,4) = -q_2;
%   W(2,1) = -q_2;
%   W(2,2) = -q_3;
%   W(2,3) = q_0;
%   W(2,4) = q_1;
%   W(3,1) = -q_3;
%   W(3,2) = q_2;
%   W(3,3) = -q_1;
%   W(3,4) = q_0;
