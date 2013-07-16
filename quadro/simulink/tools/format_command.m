%[val, start] = min(abs(time_command - time(1)));
%x_command_plot = x_command(1:length(time));
%y_command_plot = y_command(1:length(time));
%z_command_plot = z_command(1:length(time));
%psi_command_plot = psi_command(1:length(time));
x_command_plot = x_command;
y_command_plot = y_command;
z_command_plot = z_command;
psi_command_plot = psi_command;

var_struct_command = struct( ...
'command_x_lim', x_command_plot, ...
'command_y_lim', y_command_plot, ...
'command_z_lim', z_command_plot, ...
'command_psi_lim', psi_command_plot);
