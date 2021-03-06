% data.Log contains:
% data.Log.data with 
%   * xwhole 12x
%   * u_input 4x 
%   * delta_u 3x 
%   * u_z 1x 
%   * U 4x
% data.RefAltitude 
%   * roll, pitch, azimuth

% associated with simulink.mdl/Testbed\ Logger

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quantity Generally Available
%% (24x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = data.RefAttitude.time;

phi = data.RefAttitude.data(:,1);
theta = data.RefAttitude.data(:,2);
psi = data.RefAttitude.data(:,3);
var_struct_euler =  struct(...
'phi', phi, ...
'theta', theta, ...
'psi', psi );

modelx = data.Log.data(:,1);
modely = data.Log.data(:,2);
modelz = data.Log.data(:,3);

modelxdot = data.Log.data(:,4);
modelydot = data.Log.data(:,5);
modelzdot = data.Log.data(:,6);

observer_phi = data.Log.data(:,7);
observer_theta = data.Log.data(:,8);
observer_psi = data.Log.data(:,9);

observer_angularx = data.Log.data(:,10);
observer_angulary = data.Log.data(:,11);
observer_angularz = data.Log.data(:,12);

Fz = data.Log.data(:,13);
Mx = data.Log.data(:,14);
My = data.Log.data(:,15);
Mz = data.Log.data(:,16);

deltau1 = data.Log.data(:,17);
deltau2 = data.Log.data(:,18);
deltau3 = data.Log.data(:,19);

uz = data.Log.data(:,20);

U1 = data.Log.data(:,21);
U2 = data.Log.data(:,22);
U3 = data.Log.data(:,23);
U4 = data.Log.data(:,24);

real_modelx = data.Log.data(:,25);
real_modely = data.Log.data(:,26);
real_modelz = data.Log.data(:,27);

real_modelxdot = data.Log.data(:,28);
real_modelydot = data.Log.data(:,29);
real_modelzdot = data.Log.data(:,30);

real_observer_phi = data.Log.data(:,31);
real_observer_theta = data.Log.data(:,32);
real_observer_psi = data.Log.data(:,33);

real_observer_angularx = data.Log.data(:,34);
real_observer_angulary = data.Log.data(:,35);
real_observer_angularz = data.Log.data(:,36);

var_struct_modelxyz =  struct(...
'modelx', modelx,  ...
'modely', modely,  ...
'modelz', modelz  ...
);

var_struct_modelxyzdot =  struct(...
'modelxdot', modelxdot,  ...
'modelydot', modelydot,  ...
'modelzdot', modelzdot  ...
);

var_struct_observerEuler =  struct(...
'observerphi', observer_phi,  ...
'observertheta', observer_theta, ...
'observerpsi', observer_psi );


var_struct_observerAngular =  struct(...
'observerangularx', observer_angularx,  ...
'observerangulary', observer_angulary, ...
'observerangularz', observer_angularz );

var_struct_M =  struct(...
'Mx', Mx, ...
'My', My, ...
'Mz', Mz, ...,
'Fz', Fz);

var_struct_deltau =  struct(...
'deltau1', deltau1, ...
'deltau2', deltau2, ...
'deltau3', deltau3, ...
'uz', uz );

var_struct_U =  struct(...
'U1', U1, ...
'U2', U2, ...
'U3', U3, ...
'U4', U4 );


var_struct_modeloutput =  struct(...
'modelx', modelx,  ...
'modely', modely,  ...
'modelz', modelz,  ...
'observerpsi', observer_psi );


var_struct_MotFreq =  struct(...
'FreqFront', data.MotStat.data(:,3), ...
'FreqRight', data.MotStat.data(:,4), ...
'FreqRear', data.MotStat.data(:,5), ...
'FreqLeft', data.MotStat.data(:,6) );

var_struct_MotCom =  struct(...
'PWMFront', data.MotCom.data(:,1), ...
'PWMRight', data.MotCom.data(:,2), ...
'PWMRear', data.MotCom.data(:,3), ...
'PWMLeft', data.MotCom.data(:,4) );

var_struct_real_modelxyz =  struct(...
'real_modelx', real_modelx,  ...
'real_modely', real_modely,  ...
'real_modelz', real_modelz  ...
);

var_struct_real_modelxyzdot =  struct(...
'real_modelxdot', real_modelxdot,  ...
'real_modelydot', real_modelydot,  ...
'real_modelzdot', real_modelzdot  ...
);

var_struct_real_observerEuler =  struct(...
'real_observerphi', real_observer_phi,  ...
'real_observertheta', real_observer_theta, ...
'real_observerpsi', real_observer_psi );


var_struct_real_observerAngular =  struct(...
'real_observerangularx', real_observer_angularx,  ...
'real_observerangulary', real_observer_angulary, ...
'real_observerangularz', real_observer_angularz );



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quantity in Dynamic Inversion with PCH and DE
%% (20x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





refx = data.Log.data(:,37);
refy = data.Log.data(:,38);
refz = data.Log.data(:,39);
refpsi = data.Log.data(:,40);

nu1  = data.Log.data(:,41);
nu2  = data.Log.data(:,42);
nu3  = data.Log.data(:,43);
nu4  = data.Log.data(:,44);

nucorrected1  = data.Log.data(:,45);
nucorrected2  = data.Log.data(:,46);
nucorrected3  = data.Log.data(:,47);
nucorrected4  = data.Log.data(:,48);

nuhedge1  = data.Log.data(:,49);
nuhedge2  = data.Log.data(:,50);
nuhedge3  = data.Log.data(:,51);
nuhedge4  = data.Log.data(:,52);

err01 = data.Log.data(:,53);
err02 = data.Log.data(:,54);
err03 = data.Log.data(:,55);
err04 = data.Log.data(:,56);

var_struct_yref =  struct(...
'refx', refx,  ...
'refy', refy,  ...
'refz', refz,  ...
'refpsi', refpsi  ...
);

var_struct_nu = struct(...
'nu1', nu1, ...
'nu2', nu2, ...
'nu3', nu3, ...
'nu4', nu4 ...
);


var_struct_nucorrected = struct(...
'nucorrected1', nucorrected1, ...
'nucorrected2', nucorrected2, ...
'nucorrected3', nucorrected3, ...
'nucorrected4', nucorrected4 ...
);

var_struct_nuhedge = struct(...
'nuhedge1', nuhedge1, ...
'nuhedge2', nuhedge2, ...
'nuhedge3', nuhedge3, ...
'nuhedge4', nuhedge4 ...
);

var_struct_err = struct(...
'err01', err01, ...
'err02', err02, ...
'err03', err03, ...
'err04', err04);

P_modelx = data.Log.data(:,57);
P_modely = data.Log.data(:,58);
P_modelz = data.Log.data(:,59);

P_modelxdot = data.Log.data(:,60);
P_modelydot = data.Log.data(:,61);
P_modelzdot = data.Log.data(:,62);

P_observer_phi = data.Log.data(:,63);
P_observer_theta = data.Log.data(:,64);
P_observer_psi = data.Log.data(:,65);

P_observer_angularx = data.Log.data(:,66);
P_observer_angulary = data.Log.data(:,67);
P_observer_angularz = data.Log.data(:,68);

var_struct_P_modelxyz =  struct(...
'P_modelx', P_modelx,  ...
'P_modely', P_modely,  ...
'P_modelz', P_modelz  ...
);

var_struct_P_modelxyzdot =  struct(...
'P_modelxdot', P_modelxdot,  ...
'P_modelydot', P_modelydot,  ...
'P_modelzdot', P_modelzdot  ...
);

var_struct_P_observerEuler =  struct(...
'P_observerphi', P_observer_phi,  ...
'P_observertheta', P_observer_theta, ...
'P_observerpsi', P_observer_psi );


var_struct_P_observerAngular =  struct(...
'P_observerangularx', P_observer_angularx,  ...
'P_observerangulary', P_observer_angulary, ...
'P_observerangularz', P_observer_angularz );

EstError_modelx = data.Log.data(:,69);
EstError_modely = data.Log.data(:,70);
EstError_modelz = data.Log.data(:,71);

EstError_modelxdot = data.Log.data(:,72);
EstError_modelydot = data.Log.data(:,73);
EstError_modelzdot = data.Log.data(:,74);

EstError_observer_phi = data.Log.data(:,75);
EstError_observer_theta = data.Log.data(:,76);
EstError_observer_psi = data.Log.data(:,77);

EstError_observer_angularx = data.Log.data(:,78);
EstError_observer_angulary = data.Log.data(:,79);
EstError_observer_angularz = data.Log.data(:,80);

var_struct_EstError_modelxyz =  struct(...
'EstError_modelx', EstError_modelx,  ...
'EstError_modely', EstError_modely,  ...
'EstError_modelz', EstError_modelz  ...
);

var_struct_EstError_modelxyzdot =  struct(...
'EstError_modelxdot', EstError_modelxdot,  ...
'EstError_modelydot', EstError_modelydot,  ...
'EstError_modelzdot', EstError_modelzdot  ...
);

var_struct_EstError_observerEuler =  struct(...
'EstError_observerphi', EstError_observer_phi,  ...
'EstError_observertheta', EstError_observer_theta, ...
'EstError_observerpsi', EstError_observer_psi );


var_struct_EstError_observerAngular =  struct(...
'EstError_observerangularx', EstError_observer_angularx,  ...
'EstError_observerangulary', EstError_observer_angulary, ...
'EstError_observerangularz', EstError_observer_angularz );
