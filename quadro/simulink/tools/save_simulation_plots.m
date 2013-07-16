function save_simulation_plots(name, timespan)
% after initialization
if (exist('timespan')) 
    sim('full_simulation_split.mdl', timespan);
else 
    sim('full_simulation_split.mdl');
end

time = preprocess_sim(log_command.time);
xi = preprocess_sim(log_extended_states.signals(1).values);
dxidt = preprocess_sim(log_extended_states.signals(1).values);

F_wind_x = preprocess_sim(log_F_wind.signals(1).values);
F_wind_y = preprocess_sim(log_F_wind.signals(2).values);
F_wind_z = preprocess_sim(log_F_wind.signals(3).values);

command_x = preprocess_sim(log_command.signals(1).values);
command_y = preprocess_sim(log_command.signals(2).values);
command_z = preprocess_sim(log_command.signals(3).values);
command_psi = preprocess_sim(log_command.signals(4).values);

command_lim_x = preprocess_sim(log_command_limited.signals(1).values);
command_lim_y = preprocess_sim(log_command_limited.signals(2).values);
command_lim_z = preprocess_sim(log_command_limited.signals(3).values);
command_lim_psi = preprocess_sim(log_command_limited.signals(4).values);

e_x = preprocess_sim(log_e.signals(1).values);
e_y = preprocess_sim(log_e.signals(2).values);
e_z = preprocess_sim(log_e.signals(3).values);
e_psi = preprocess_sim(log_e.signals(4).values);

nu_x = preprocess_sim(log_nu.signals(1).values);
nu_y = preprocess_sim(log_nu.signals(2).values);
nu_z = preprocess_sim(log_nu.signals(3).values);
nu_psi = preprocess_sim(log_nu.signals(4).values);

nu_hedge_x = preprocess_sim(log_nu_hedge.signals(1).values);
nu_hedge_y = preprocess_sim(log_nu_hedge.signals(2).values);
nu_hedge_z = preprocess_sim(log_nu_hedge.signals(3).values);
nu_hedge_psi = preprocess_sim(log_nu_hedge.signals(4).values);

Fz = preprocess_sim(log_u_real.signals(1).values);
Mx = preprocess_sim(log_u_real.signals(2).values);
My = preprocess_sim(log_u_real.signals(3).values);
Mz = preprocess_sim(log_u_real.signals(4).values);

x = preprocess_sim(squeeze(log_xwhole.signals.values(1,1,:)));
y = preprocess_sim(squeeze(log_xwhole.signals.values(2,1,:)));
z = preprocess_sim(squeeze(log_xwhole.signals.values(3,1,:)));
dx = preprocess_sim(squeeze(log_xwhole.signals.values(4,1,:)));
dy = preprocess_sim(squeeze(log_xwhole.signals.values(5,1,:)));
dz = preprocess_sim(squeeze(log_xwhole.signals.values(6,1,:)));
phi = preprocess_sim(squeeze(log_xwhole.signals.values(7,1,:)));
theta = preprocess_sim(squeeze(log_xwhole.signals.values(8,1,:)));
psi = preprocess_sim(squeeze(log_xwhole.signals.values(9,1,:)));
angularx = preprocess_sim(squeeze(log_xwhole.signals.values(10,1,:)));
angulary = preprocess_sim(squeeze(log_xwhole.signals.values(11,1,:)));
angularz = preprocess_sim(squeeze(log_xwhole.signals.values(12,1,:)));

refx = preprocess_sim(log_y_ref.signals(1).values);
refy = preprocess_sim(log_y_ref.signals(2).values);
refz = preprocess_sim(log_y_ref.signals(3).values);
refpsi = preprocess_sim(log_y_ref.signals(4).values);

plot(time,x); matlab2tikz([name '_x.tikz']); close;
plot(time,y); matlab2tikz([name '_y.tikz']); close;
plot(time,z); matlab2tikz([name '_z.tikz']); close;

plot(time,F_wind_x); matlab2tikz([name '_F_wind_x.tikz']); close;
plot(time,F_wind_y); matlab2tikz([name '_F_wind_y.tikz']); close;
plot(time,F_wind_z); matlab2tikz([name '_F_wind_z.tikz']); close;


plot(time,dx); matlab2tikz([name '_dx.tikz']); close;
plot(time,dy); matlab2tikz([name '_dy.tikz']); close;
plot(time,dz); matlab2tikz([name '_dz.tikz']); close;

plot(time,phi); matlab2tikz([name '_phi.tikz']); close;
plot(time,theta); matlab2tikz([name '_theta.tikz']); close;
plot(time,psi); matlab2tikz([name '_psi.tikz']); close;

plot(time,angularx); matlab2tikz([name '_angularx.tikz']); close;
plot(time,angulary); matlab2tikz([name '_angulary.tikz']); close;
plot(time,angularz); matlab2tikz([name '_angularz.tikz']); close;

plot(time,refx); matlab2tikz([name '_refx.tikz']); close;
plot(time,refy); matlab2tikz([name '_refy.tikz']); close;
plot(time,refz); matlab2tikz([name '_refz.tikz']); close;
plot(time,refpsi); matlab2tikz([name '_refpsi.tikz']); close;

plot(time,Fz); matlab2tikz([name '_Fz.tikz']); close;
plot(time,Mx); matlab2tikz([name '_Mx.tikz']); close;
plot(time,My); matlab2tikz([name '_My.tikz']); close;
plot(time,Mz); matlab2tikz([name '_Mz.tikz']); close;

plot(time,nu_x); matlab2tikz([name '_nu_x.tikz']); close;
plot(time,nu_y); matlab2tikz([name '_nu_y.tikz']); close;
plot(time,nu_z); matlab2tikz([name '_nu_z.tikz']); close;
plot(time,nu_psi); matlab2tikz([name '_nu_psi.tikz']); close;

plot(time,nu_hedge_x); matlab2tikz([name '_nu_hedge_x.tikz']); close;
plot(time,nu_hedge_y); matlab2tikz([name '_nu_hedge_y.tikz']); close;
plot(time,nu_hedge_z); matlab2tikz([name '_nu_hedge_z.tikz']); close;
plot(time,nu_hedge_psi); matlab2tikz([name '_nu_hedge_psi.tikz']); close;

plot(time,command_x); matlab2tikz([name '_command_x.tikz']); close;
plot(time,command_y); matlab2tikz([name '_command_y.tikz']); close;
plot(time,command_z); matlab2tikz([name '_command_z.tikz']); close;
plot(time,command_psi); matlab2tikz([name '_command_psi.tikz']); close;

plot(time,command_lim_x); matlab2tikz([name '_command_lim_x.tikz']); close;
plot(time,command_lim_y); matlab2tikz([name '_command_lim_y.tikz']); close;
plot(time,command_lim_z); matlab2tikz([name '_command_lim_z.tikz']); close;
plot(time,command_lim_psi); matlab2tikz([name '_command_lim_psi.tikz']); close;

plot(time,e_x); matlab2tikz([name '_e_x.tikz']); close;
plot(time,e_y); matlab2tikz([name '_e_y.tikz']); close;
plot(time,e_z); matlab2tikz([name '_e_z.tikz']); close;
plot(time,e_psi); matlab2tikz([name '_e_psi.tikz']); close;

hold on;
plot(time, x);  plot(time,command_x); matlab2tikz([name '_x_and_command_x.tikz']); close;
hold on;
plot(time, y);  plot(time,command_y); matlab2tikz([name '_y_and_command_y.tikz']); close;
hold on;
plot(time, z);  plot(time,command_z); matlab2tikz([name '_z_and_command_z.tikz']); close;
hold on;
plot(time, psi); plot(time,command_psi); matlab2tikz([name '_psi_and_command_psi.tikz']); close;
hold on;
plot(time, x);  plot(time,refx); matlab2tikz([name '_x_and_refx.tikz']); close;
hold on;
plot(time, y);  plot(time,refy); matlab2tikz([name '_y_and_refy.tikz']); close;
hold on;
plot(time, z);  plot(time,refz); matlab2tikz([name '_z_and_refz.tikz']); close;
hold on;
plot(time, psi); plot(time,refpsi); matlab2tikz([name '_psi_and_refpsi.tikz']); close;
hold on;
plot(time, x);  plot(time,command_lim_x); matlab2tikz([name '_x_and_command_lim_x.tikz']); close;
hold on;
plot(time, y);  plot(time,command_lim_y); matlab2tikz([name '_y_and_command_lim_y.tikz']); close;
hold on;
plot(time, z);  plot(time,command_lim_z); matlab2tikz([name '_z_and_command_lim_z.tikz']); close;
hold on;
plot(time, psi); plot(time,command_lim_psi); matlab2tikz([name '_psi_and_command_lim_psi.tikz']); close;
hold off;


end
