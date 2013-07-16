% load data according to the two cell arrays matfiles and shortnames
for jj=1:length(matfiles)
    load(char(matfiles(jj)));

    name = regexprep(char(matfiles(jj)), '.mat','');
    structlist = whos('var_struct*');
    try 
        eval(name)
        get_command
        format_command
    catch err
        if (strcmp(err.identifier,'MATLAB:UndefinedFunction'))
            x_command_plot = zeros(1,300*100);
            y_command_plot = zeros(1,300*100);
            z_command_plot = zeros(1,300*100);
            psi_command_plot = zeros(1,300*100);

            var_struct_command = struct( ...
            'command_x_lim', x_command_plot, ...
            'command_y_lim', y_command_plot, ...
            'command_z_lim', z_command_plot, ...
            'command_psi_lim', psi_command_plot);
            disp 'error in loop'
        end
    end
    structlist = whos('var_struct*');


    eval([char(shortnames{jj}) '= data;' ])
    for ii=1:length(structlist)
        eval([char(shortnames{jj}) '.' structlist(ii).name '='  structlist(ii).name ';' ])
    end
end
