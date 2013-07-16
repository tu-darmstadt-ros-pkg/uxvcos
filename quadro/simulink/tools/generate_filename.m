function filename_string =  generate_filename( string )

datestring = '';
datenum = clock();

format = '%02d';
for ii = 1:length(datenum)
	datestring = strcat(datestring, num2str(floor(datenum(ii)), format));
end

filename_string = strcat('quadro-', string);
filename_string = strcat(filename_string, '-');
filename_string = strcat(filename_string, datestring);
filename_string = strcat(filename_string, '.mat');
filename_string = strcat('./data/', filename_string);

end
